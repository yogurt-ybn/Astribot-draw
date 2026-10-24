"""Conservative sphere broad phase, followed by the original MuJoCo distance query."""
import numpy as np


class CollisionChecker:
    def __init__(self, model, pairs=None):
        import mujoco
        self.mujoco, self.model = mujoco, model
        self.pairs = (list(pairs) if pairs is not None else
                      [(int(a), int(b), 'geometry') for a, b in zip(model.pair_geom1, model.pair_geom2)])
        self.ids = np.asarray([(p[0], p[1]) for p in self.pairs], dtype=int).reshape(-1, 2)
        self.radii = model.geom_rbound[self.ids].sum(axis=1)
        self.planes = np.any(model.geom_type[self.ids] == mujoco.mjtGeom.mjGEOM_PLANE, axis=1)
        self.categories = tuple(dict.fromkeys(p[2] for p in self.pairs))
        self.names = tuple(model.geom(i).name for i in range(model.ngeom))

    def distances(self, data):
        delta = data.geom_xpos[self.ids[:, 0]]-data.geom_xpos[self.ids[:, 1]]
        lower_bound = np.linalg.norm(delta, axis=1)-self.radii
        candidates = np.flatnonzero(self.planes | (lower_bound < .05+1e-9))
        for i in candidates:
            a, b, category = self.pairs[i]
            yield a, b, category, float(self.mujoco.mj_geomDistance(self.model, data, a, b, .05, None))

    def minimum_distance(self, data):
        """Same queries/threshold as check(), without per-category report allocation."""
        return min(.05, min((distance for _, _, _, distance in self.distances(data)), default=.05))

    def check(self, data):
        best = dict(distance_m=.05, minimum_distance_m=.05, geom1=None, geom2=None, category=None)
        categories = {name: dict(distance_m=.05, geom1=None, geom2=None) for name in self.categories}
        for a, b, category, distance in self.distances(data):
            if distance < categories[category]['distance_m']:
                record = dict(distance_m=distance, geom1=self.names[a], geom2=self.names[b])
                categories[category] = record
                if distance < best['distance_m']:
                    best.update(record, minimum_distance_m=distance, category=category)
        best['category_minima'] = categories
        return best


def checked_transfer(m, c, start, goal, torso, home, sub, rng):
    """The upstream deterministic RRT-Connect with identical edge spacing and clearance."""
    import mujoco
    checker = CollisionChecker(m, [(a, b, 'transfer') for a, b in sub])
    data = mujoco.MjData(m)

    def valid(q):
        data.qpos[:] = home
        data.qpos[:4] = torso
        data.qpos[c.qids] = q
        mujoco.mj_kinematics(m, data)
        return checker.minimum_distance(data) >= .003

    def edge(a, b):
        return all(valid(a+r*(b-a)) for r in np.linspace(0, 1, max(2, int(np.ceil(np.max(abs(b-a))/.01))+1)))

    if not valid(start) or not valid(goal):
        raise RuntimeError('Measured start or entry goal fails the 3 mm transfer-clearance gate')
    if edge(start, goal):
        return [start, goal]
    trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]

    def grow(tree, q):
        nodes, parents = tree
        ix = int(np.argmin([np.linalg.norm(x-q) for x in nodes]))
        a = nodes[ix]
        v, dist = q-a, np.linalg.norm(q-a)
        b = q if dist < .25 else a+v*.25/dist
        if not edge(a, b):
            return None
        nodes.append(b.copy())
        parents.append(ix)
        return len(nodes)-1

    def back(tree, k):
        out = []
        while k >= 0:
            out.append(tree[0][k])
            k = tree[1][k]
        return out[::-1]

    for it in range(1500):
        k = it % 2
        q = goal if rng.random() < .15 else np.clip(start+rng.random()*(goal-start)+rng.normal(0, .7, len(start)), c.lo, c.hi)
        ia = grow(trees[k], q)
        if ia is None:
            continue
        q = trees[k][0][ia]
        for _ in range(70):
            ib = grow(trees[1-k], q)
            if ib is None:
                break
            if np.linalg.norm(trees[1-k][0][ib]-q) < 1e-8:
                path = back(trees[k], ia)+back(trees[1-k], ib)[-2::-1]
                if k:
                    path = path[::-1]
                slim, i = [path[0]], 0
                while i < len(path)-1:
                    j = len(path)-1
                    while j > i+1 and not edge(path[i], path[j]):
                        j -= 1
                    slim.append(path[j])
                    i = j
                print('RRT_PASS measured-start', it, len(slim), flush=True)
                return slim
    raise RuntimeError('No checked entry path from measured state within the deterministic RRT budget')

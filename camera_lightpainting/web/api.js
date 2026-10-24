/** Transport boundary: UI modules do not construct pipeline requests. */
const ROOT = '/api/v1';

async function request(path, data) {
  const response = await fetch(ROOT + path, {
    method: data === undefined ? 'GET' : 'POST',
    headers: data === undefined ? {} : { 'Content-Type': 'application/json' },
    body: data === undefined ? undefined : JSON.stringify(data),
    signal: AbortSignal.timeout(10000),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || '操作失败，请重试');
  return payload;
}

export const api = {
  state: () => request('/state'),
  devices: () => request('/devices'),
  settings: changes => request('/settings', changes),
  action: (action, details = {}) => request('/action', { action, ...details }),
  preview: view => `${ROOT}/preview/${view}.jpg?t=${Date.now()}`,
};

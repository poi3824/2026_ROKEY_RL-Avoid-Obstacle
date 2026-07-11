const API_BASE = import.meta.env.VITE_API_BASE;

async function apiGet(path) {
  const res = await fetch(`${API_BASE}${path}`);
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.error || data.message || `${path} -> HTTP ${res.status}`);
  }
  return data;
}

export const fetchHealth = () => apiGet("/api/health");
export const fetchDbSummary = () => apiGet("/api/db/summary");
export const fetchPickAttempts = (limit = 50) => apiGet(`/api/db/pick_attempts?limit=${limit}`);
export const fetchVoiceEvents = (limit = 50) => apiGet(`/api/db/voice_events?limit=${limit}`);
export const fetchWorldmapScans = (limit = 30) => apiGet(`/api/db/worldmap_scans?limit=${limit}`);
export const fetchWorldmapList = () => apiGet("/api/worldmap/list");
export const fetchWorldmapLatest = () => apiGet("/api/worldmap/latest");
export const fetchWorldmapObstacles = (scanId) => apiGet(`/api/worldmap/${encodeURIComponent(scanId)}/obstacles`);
export const fetchWorldmapPoints = (scanId) => apiGet(`/api/worldmap/${encodeURIComponent(scanId)}/points`);

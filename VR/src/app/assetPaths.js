export function resolveAssetBaseUrl(modelUrl) {
  return new URL('.', new URL(modelUrl, window.location.href)).href;
}

export function resolveRobotMeshUrls(robotJson, assetBaseUrl) {
  const normalized = structuredCloneCompat(robotJson);
  const baseUrl = new URL(assetBaseUrl, window.location.href);

  normalized.links?.forEach((link) => {
    const geometry = link.visual?.geometry;
    if (!geometry || geometry.type !== 'mesh' || !geometry.url) {
      return;
    }

    geometry.url = new URL(geometry.url, baseUrl).href;
  });

  return normalized;
}

function structuredCloneCompat(value) {
  if (typeof structuredClone === 'function') {
    return structuredClone(value);
  }

  return JSON.parse(JSON.stringify(value));
}

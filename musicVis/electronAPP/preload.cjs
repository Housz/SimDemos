// preload.cjs（CommonJS，避免 ESM 兼容性问题）
const { contextBridge, desktopCapturer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  // 列出可抓取的桌面/窗口源（用于 Windows 系统音频）
  listSources: async () => {
    const sources = await desktopCapturer.getSources({ types: ['screen', 'window'] });
    return sources.map(s => ({ id: s.id, name: s.name }));
  }
});

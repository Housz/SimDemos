// preload.js
import { contextBridge, ipcRenderer } from 'electron';

contextBridge.exposeInMainWorld('electronAPI', {
  listSources: () => ipcRenderer.invoke('list-capture-sources')
});

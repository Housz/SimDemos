// main.js
import { app, BrowserWindow } from 'electron';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

// 不在后台限速 + 自动播放
app.commandLine.appendSwitch('disable-renderer-backgrounding');
app.commandLine.appendSwitch('disable-background-timer-throttling');
app.commandLine.appendSwitch('autoplay-policy', 'no-user-gesture-required');

function createWindow() {
  const win = new BrowserWindow({
    width: 1280,
    height: 720,
    backgroundColor: '#0a0b10',
    webPreferences: {
      preload: path.join(__dirname, 'preload.cjs'), // ★ 使用 CommonJS 预加载
      contextIsolation: true,
      nodeIntegration: false,
      backgroundThrottling: false // ★ 渲染进程后台也不停帧
    }
  });

  win.loadFile('index.html');
  // win.setAlwaysOnTop(true); // 需要时置顶
}

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });

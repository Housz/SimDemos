import { APP_CONFIG } from './config.js';
import { RoadheaderVRApp } from './app/RoadheaderVRApp.js';

const app = new RoadheaderVRApp({
  canvas: document.querySelector('#scene-canvas'),
  fileInput: document.querySelector('#robot-file-input'),
  uploadButton: document.querySelector('#upload-button'),
  targetColorButton: document.querySelector('#target-color-button'),
  statusElement: document.querySelector('#status'),
  config: APP_CONFIG
});

app.init().catch((error) => {
  console.error(error);
  app.setStatus(`Startup failed: ${error.message}`);
});

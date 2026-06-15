// server.js —— CommonJS 版本
const https = require('https');
const fs = require('fs');
const path = require('path');
const express = require('express');

// 当前运行目录（不要再用 __dirname 覆盖系统变量）
const rootDir = process.cwd();

// 证书路径
const options = {
  key: fs.readFileSync(path.join(rootDir, 'key.pem')),
  cert: fs.readFileSync(path.join(rootDir, 'cert.pem')),
};

const app = express();

// 静态文件目录
const STATIC_DIR = path.join(rootDir, '');
app.use(express.static(STATIC_DIR));

// 访问 /1.html 时，跳转到 /robot/demos/roadheader/vr.html
app.get('/1.html', (req, res) => {
  // 如果你要 "重定向"（浏览器地址栏会变）用：
  // res.redirect('/robot/demos/roadheader/vr.html');

  // 如果你要 "内部映射"（地址栏不变，内容是 vr.html），用：
  res.sendFile(path.join(STATIC_DIR, 'robot/demos/roadheader/vr.html'));
});

// 启动 HTTPS 服务
const PORT = 1000;
https.createServer(options, app).listen(PORT, '0.0.0.0', () => {
  console.log(`✅ HTTPS server running at https://192.168.31.61:${PORT}`);
  console.log(`📂 Serving files from: ${STATIC_DIR}`);
});

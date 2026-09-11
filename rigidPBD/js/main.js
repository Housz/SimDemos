// ============================================================================
// main.js —— 页面入口：构建演示框架、渲染场景列表、处理场景切换
//
// 运行方式（ES module 不能经 file:// 加载，必须起一个静态服务）：
//   cd rigidPBD && python -m http.server 8000
//   浏览器打开 http://localhost:8000/
// ============================================================================

import { Harness } from './harness.js';
import { scenes } from './scenes/registry.js';

const view = document.getElementById('view');
const listEl = document.getElementById('scene-list');
const infoEl = document.getElementById('scene-info');

const harness = new Harness(view);

// 调试 HUD 与能量面板（元素在 index.html 中，可选）
const energyPanel = document.getElementById('energy-panel');
energyPanel.style.display = 'none'; // 默认关闭，由 GUI 的「能量曲线」开关打开
harness.attachHUD({
    fps: document.getElementById('fps'),
    stats: document.getElementById('stats'),
    info: document.getElementById('live-info'), // 实时读数（与侧栏的静态说明分开）
    energy: document.getElementById('energy'),
});

// 便于在浏览器控制台里把玩：harness.world / harness.params
window.harness = harness;

// ---------------------------------------------------------------------------
// 场景列表与切换
// ---------------------------------------------------------------------------
let activeIndex = 0;

function selectScene(i) {
    activeIndex = i;
    const def = scenes[i];
    harness.setScene(def);
    for (let k = 0; k < listEl.children.length; k++) {
        listEl.children[k].classList.toggle('active', k === i);
    }
    infoEl.innerHTML = `<b>${def.name}</b><br>${def.desc}`;
    // 每个场景的相机预设不同，切换后立即同步一次
    document.title = `${def.name} —— XPBD 刚体模拟`;
}

scenes.forEach((def, i) => {
    const btn = document.createElement('button');
    btn.className = 'scene-btn';
    btn.innerHTML = `${def.name}<span class="tag">${def.tag || ''}</span>`;
    btn.addEventListener('click', () => selectScene(i));
    listEl.appendChild(btn);
});

selectScene(0);

// 数字键 1..9 快速切换
window.addEventListener('keydown', (e) => {
    const n = parseInt(e.key, 10);
    if (n >= 1 && n <= scenes.length) selectScene(n - 1);
});

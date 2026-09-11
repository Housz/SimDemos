// 无头测试的启动引导：
//   1) 若 test/three.module.js 不存在，则从 unpkg 下载（与 index.html 的 import map 同版本）
//   2) 注册 loader.mjs 的 'three' 解析钩子
//
// 用法（在 rigidPBD/ 目录下）：
//   node --import ./test/register.mjs test/physics.test.mjs
import { register } from 'node:module';
import { existsSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const THREE_VERSION = '0.156.0';
const target = fileURLToPath(new URL('./three.module.js', import.meta.url));

if (!existsSync(target)) {
    const url = `https://unpkg.com/three@${THREE_VERSION}/build/three.module.js`;
    console.log(`[test] 下载 three.js r${THREE_VERSION.split('.')[1]}：${url}`);
    const res = await fetch(url);
    if (!res.ok) throw new Error(`下载 three.module.js 失败：HTTP ${res.status}`);
    writeFileSync(target, Buffer.from(await res.arrayBuffer()));
    console.log(`[test] 已缓存到 ${target}`);
}

register('./loader.mjs', import.meta.url);

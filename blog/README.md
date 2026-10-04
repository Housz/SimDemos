# 博客交互演示

原生单页 HTML，无需安装依赖或构建。每页可以独立打开，也可以放进博客的 iframe。

## 页面

- `laplacian.html`：曲面、坐标方向截线与五点差分。比较碗面、倒碗面、斜平面、鞍面，以及用于观察差分误差的四次函数。
- `heat.html`：在二维 SVG 格网上编辑温度初值，并观察显式热扩散。

建议在文章的“二维：把两个方向的结果相加”中嵌入曲面页；“五点格式”或“邻域平均”可再次引用同一页。“热传导与时间更新”中嵌入热扩散页。

合并到 GitHub Pages 发布分支后，页面地址为：

- https://simdemos.housz.cn/blog/laplacian.html
- https://simdemos.housz.cn/blog/heat.html

```html
<iframe src="https://simdemos.housz.cn/blog/laplacian.html"
  title="拉普拉斯：曲面与五点差分" width="100%" height="480"
  style="border:0" loading="lazy" allowfullscreen></iframe>
<iframe src="https://simdemos.housz.cn/blog/heat.html"
  title="二维热传导交互实验" width="100%" height="480"
  style="border:0" loading="lazy" allowfullscreen></iframe>
```

窄屏时页面可以纵向滚动；每页提供“单独打开”链接。触摸拖动需要主动开启，默认保留文章滚动。无需 iframe 自动测高或父页面脚本。

## 本地运行与检查

在仓库根目录启动任意静态文件服务，例如：

```sh
python -m http.server 8000
node --test blog/tests/*.test.cjs
```

打开 `http://localhost:8000/blog/laplacian.html` 和 `http://localhost:8000/blog/heat.html`。

测试直接抽取 HTML 中的求解脚本，不维护另一份计算公式。根目录原有 `npm test` 是占位脚本；上述命令只检查本目录的演示。

另外提供基于 jsdom 的真实 DOM 事件检查（测试依赖独立放在 `blog/tests`，不影响演示页面）：

```sh
npm ci --prefix blog/tests --ignore-scripts
npm test --prefix blog/tests
```

它覆盖预设、参数、绘制、键盘选择、播放/暂停、单步、恢复初值和 SVG 视角按钮。纯计算测试与 DOM 模拟不等于浏览器验收：合并前还应在支持 WebGL 的浏览器检查正常 3D 渲染、CDN 失败、桌面与手机的 480px iframe 布局、触摸滚动、视角拖动及涂色命中。

## 依赖与降级

曲面页仅 3D 渲染部分通过 CDN 加载固定版本 Three.js 0.156.0 和同版本 OrbitControls。数值计算与 SVG 备用视图不依赖 CDN；离线、CDN 失败或 WebGL 不可用时仍能使用。热扩散页无外部依赖。

数学约定、边界条件和参数含义直接显示在各页面中。此演示用于理解离散算子，不是实际材料或工程问题的模拟工具。

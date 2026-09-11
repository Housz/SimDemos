// 解析钩子：把裸模块名 'three' 映射到 test/ 目录下本地缓存的 three.module.js。
// 浏览器端用 import map 从 unpkg 加载；无头测试不能直接 import https，故做此映射。
export function resolve(specifier, context, next) {
    if (specifier === 'three') {
        return { url: new URL('./three.module.js', import.meta.url).href, shortCircuit: true };
    }
    return next(specifier, context);
}

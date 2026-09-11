// ============================================================================
// registry.js —— 场景注册表
//
// 每个场景模块导出同一个结构：
//   {
//     id, name, tag, desc,            // 名称、侧栏标签、说明（中文）
//     camera: { position, target },   // 相机预设
//     create(harness) → { preStep, postStep, onGUI, dispose },
//     buildInfo(harness) → HTML 字符串（右上角实时读数，可选）
//   }
//
// preStep(dt)  在 world.step() 之前调用 —— 设置外力/力矩、推进马达目标
// postStep(dt) 在 world.step() 之后调用 —— 读取状态、驱动运动学物体
// ============================================================================

import springBoxes from './sceneSpringBoxes.js';
import pendula from './scenePendula.js';
import jointTypes from './sceneJointTypes.js';
import robot from './sceneRobot.js';
import rope from './sceneRope.js';
import marbles from './sceneMarbles.js';
import coin from './sceneCoin.js';
import boxStack from './sceneBoxStack.js';

export const scenes = [
    springBoxes,
    pendula,
    jointTypes,
    robot,
    rope,
    marbles,
    coin,
    boxStack,
];

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { OculusHandModel } from 'three/addons/webxr/OculusHandModel.js';
import { VRButton } from 'three/addons/webxr/VRButton.js';
import { XRControllerModelFactory } from 'three/addons/webxr/XRControllerModelFactory.js';

import { LeverPanel } from './LeverPanel.js';
import { resolveAssetBaseUrl, resolveRobotMeshUrls } from './assetPaths.js';
import { robotConstraintHandler } from '../robot/robotConstraintHandler.js';
import { robotCreator } from '../robot/robotCreator.js';
import { robotGUICreator } from '../robot/robotGUICreator.js';
import { robotParser } from '../robot/robotParser.js';
import { robotRHIKHandler } from '../robot/robotRHIKHandler.js';
import { robotWorkSpaceCreator } from '../robot/robotWorkSpaceCreator.js';

const ARC_SEGMENTS = 200;

class ControllerPointer {
  constructor(controller) {
    this.controller = controller;
    this.position = new THREE.Vector3();
    this.box = new THREE.Box3();

    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(0.008, 16, 16),
      new THREE.MeshBasicMaterial({
        color: 0x00ffff,
        transparent: true,
        opacity: 0.12
      })
    );

    tip.visible = false;
    tip.position.set(0, 0, -0.025);
    controller.add(tip);
    this.tip = tip;
  }

  getPointerPosition() {
    return this.tip.getWorldPosition(this.position);
  }

  intersectBoxObject(object) {
    this.box.setFromObject(object);
    return this.box.containsPoint(this.getPointerPosition());
  }
}

export class RoadheaderVRApp {
  constructor({
    canvas,
    fileInput,
    uploadButton,
    targetColorButton,
    statusElement,
    config
  }) {
    if (!canvas) {
      throw new Error('Missing scene canvas.');
    }

    this.canvas = canvas;
    this.fileInput = fileInput;
    this.uploadButton = uploadButton;
    this.targetColorButton = targetColorButton;
    this.statusElement = statusElement;
    this.config = config;

    this.clock = new THREE.Clock();
    this.constraintGuiMap = new Map();
    this.pointers = [];
    this.isTargetGreen = true;
    this.trajectoryCurve = null;
    this.trajectoryPositions = new Float32Array(ARC_SEGMENTS * 3);

    this.robot = null;
    this.robotModel = null;
    this.robotGui = null;
    this.workspaceGui = null;
    this.constraintGui = null;
    this.controlGui = null;
    this.tunnelGui = null;
  }

  async init() {
    this.setupScene();
    this.setupXR();
    this.setupUiActions();
    this.createControlGui();

    this.leverPanel = new LeverPanel({
      renderer: this.renderer,
      panelGroup: this.panelGroup,
      pointers: this.pointers,
      panelDistance: this.config.panel.distance,
      panelYOffset: this.config.panel.yOffset,
      hydraulicSpeed: this.config.hydraulics.speed,
      leverDeadzone: this.config.hydraulics.deadzone
    });

    this.renderer.setAnimationLoop(() => this.render());

    this.loadTunnelModel().catch((error) => {
      console.error(error);
      this.setStatus(`Tunnel model failed: ${error.message}`);
    });

    await this.loadRobotFromUrl(this.config.robotModelUrl);
  }

  setStatus(message) {
    if (this.statusElement) {
      this.statusElement.textContent = message;
    }
  }

  setupScene() {
    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      canvas: this.canvas
    });
    this.renderer.shadowMap.enabled = true;
    this.renderer.xr.enabled = true;

    this.camera = new THREE.PerspectiveCamera(75, 2, 0.01, 500);
    this.camera.position.set(0, 5.5, 5);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xaaccff);
    this.scene.fog = new THREE.Fog(0xa0a0a0, 10, 200);

    this.worldGroup = new THREE.Group();
    this.scene.add(this.worldGroup);

    this.panelGroup = new THREE.Group();
    this.scene.add(this.panelGroup);

    this.orbitControls = new OrbitControls(this.camera, this.renderer.domElement);
    this.transformControl = new TransformControls(this.camera, this.renderer.domElement);
    this.transformControl.visible = false;
    this.transformControl.addEventListener('dragging-changed', (event) => {
      this.orbitControls.enabled = !event.value;
    });
    this.transformControl.addEventListener('change', () => {
      if (this.robot && this.robotModel) {
        robotRHIKHandler(this.targetMesh, this.robot, this.robotModel);
      }
    });
    this.scene.add(this.transformControl);

    const pmremGenerator = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmremGenerator.fromScene(new RoomEnvironment(), 0.04).texture;

    this.worldGroup.add(new THREE.AmbientLight(0xffffff, 1));

    const keyLight = new THREE.DirectionalLight(0xffffff, 2);
    keyLight.position.set(5, 5, 10);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.set(1024, 1024);
    keyLight.shadow.camera.near = 0.5;
    keyLight.shadow.camera.far = 50;
    this.worldGroup.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0xffffff, 1.5);
    fillLight.position.set(5, 10, -10);
    fillLight.castShadow = true;
    fillLight.shadow.mapSize.set(1024, 1024);
    fillLight.shadow.camera.near = 0.5;
    fillLight.shadow.camera.far = 50;
    this.worldGroup.add(fillLight);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(200, 200),
      new THREE.MeshStandardMaterial({ color: 0xcccccc, depthWrite: false })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.worldGroup.add(ground);

    const grid = new THREE.GridHelper(200, 200);
    grid.material.opacity = 0.8;
    grid.material.transparent = true;
    this.worldGroup.add(grid);

    this.targetMaterial = new THREE.MeshStandardMaterial({
      color: 0x03ee00,
      transparent: true,
      opacity: 0.5
    });
    this.targetMesh = new THREE.Mesh(new THREE.SphereGeometry(0.7, 16, 8), this.targetMaterial);
    this.targetMesh.visible = false;
    this.worldGroup.add(this.targetMesh);
    this.transformControl.attach(this.targetMesh);

    this.keyGroup = new THREE.Group();
    this.worldGroup.add(this.keyGroup);
    this.createTrajectoryLine();

    window.addEventListener('resize', () => this.onResize());
  }

  createTrajectoryLine() {
    this.lineGeometry = new LineGeometry();
    this.lineGeometry.setPositions(this.trajectoryPositions);

    this.lineMaterial = new LineMaterial({
      color: 0x008811,
      linewidth: 0.7
    });
    this.updateLineResolution();

    this.trajectoryLineMesh = new Line2(this.lineGeometry, this.lineMaterial);
    this.trajectoryLineMesh.computeLineDistances();
    this.worldGroup.add(this.trajectoryLineMesh);
  }

  setupXR() {
    const controllerModelFactory = new XRControllerModelFactory();

    this.controller1 = this.renderer.xr.getController(0);
    this.controller2 = this.renderer.xr.getController(1);
    this.scene.add(this.controller1, this.controller2);

    const controllerGrip1 = this.renderer.xr.getControllerGrip(0);
    controllerGrip1.add(controllerModelFactory.createControllerModel(controllerGrip1));
    this.scene.add(controllerGrip1);

    const controllerGrip2 = this.renderer.xr.getControllerGrip(1);
    controllerGrip2.add(controllerModelFactory.createControllerModel(controllerGrip2));
    this.scene.add(controllerGrip2);

    this.hand1 = this.renderer.xr.getHand(0);
    this.handModel1 = new OculusHandModel(this.hand1);
    this.hand1.add(this.handModel1);
    this.scene.add(this.hand1);

    this.hand2 = this.renderer.xr.getHand(1);
    this.handModel2 = new OculusHandModel(this.hand2);
    this.hand2.add(this.handModel2);
    this.scene.add(this.hand2);

    this.pointers.push(
      this.handModel1,
      this.handModel2,
      new ControllerPointer(this.controller1),
      new ControllerPointer(this.controller2)
    );

    document.body.appendChild(
      VRButton.createButton(this.renderer, {
        optionalFeatures: ['hand-tracking']
      })
    );

    this.renderer.xr.addEventListener('sessionstart', () => {
      const start = this.config.userStart;
      const position = new THREE.Vector3(...start.position);
      const yaw = THREE.MathUtils.degToRad(start.yawDegrees);
      this.worldGroup.position.copy(position).multiplyScalar(-1);
      this.worldGroup.rotation.set(0, -yaw, 0);
    });

    this.renderer.xr.addEventListener('sessionend', () => {
      this.worldGroup.position.set(0, 0, 0);
      this.worldGroup.rotation.set(0, 0, 0);
    });
  }

  setupUiActions() {
    this.uploadButton?.addEventListener('click', () => {
      this.fileInput?.click();
    });

    this.fileInput?.addEventListener('change', async (event) => {
      const file = event.target.files?.[0];
      event.target.value = '';

      if (!file) {
        return;
      }

      try {
        await this.loadRobotFile(file);
      } catch (error) {
        console.error(error);
        this.setStatus(`Upload failed: ${error.message}`);
      }
    });

    this.targetColorButton?.addEventListener('click', () => {
      this.isTargetGreen = !this.isTargetGreen;
      this.targetMaterial.color.set(this.isTargetGreen ? 0x00ff00 : 0xffff00);
    });
  }

  async loadTunnelModel() {
    const gltf = await new GLTFLoader().loadAsync(this.config.tunnelModelUrl);
    const model = gltf.scene;

    model.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    model.position.set(-4, model.position.y, 1.6);
    this.worldGroup.add(model);

    this.tunnelGui = new GUI();
    this.tunnelGui.domElement.style.position = 'absolute';
    this.tunnelGui.domElement.style.left = '0px';
    this.tunnelGui.domElement.style.opacity = '0.8';
    this.tunnelGui.domElement.style.top = '650px';
    this.tunnelGui.title('Tunnel Position');

    const positionFolder = this.tunnelGui.addFolder('Position');
    const position = {
      x: model.position.x,
      y: model.position.y,
      z: model.position.z
    };
    positionFolder.add(position, 'x', -10, 10).onChange((value) => {
      model.position.x = value;
    });
    positionFolder.add(position, 'y', -10, 10).onChange((value) => {
      model.position.y = value;
    });
    positionFolder.add(position, 'z', -10, 10).onChange((value) => {
      model.position.z = value;
    });
    positionFolder.open();
  }

  async loadRobotFromUrl(modelUrl) {
    this.setStatus('Loading roadheader robot...');

    const response = await fetch(modelUrl);
    if (!response.ok) {
      throw new Error(`Failed to load ${modelUrl}: ${response.status}`);
    }

    const robotJson = await response.json();
    await this.loadRobotFromDefinition(robotJson, resolveAssetBaseUrl(modelUrl));
  }

  async loadRobotFile(file) {
    this.setStatus(`Loading ${file.name}...`);
    const robotJson = JSON.parse(await file.text());
    const assetBaseUrl = this.config.robotAssetBaseUrl || resolveAssetBaseUrl(this.config.robotModelUrl);
    await this.loadRobotFromDefinition(robotJson, assetBaseUrl);
    this.setStatus(`Loaded ${file.name}`);
  }

  async loadRobotFromDefinition(robotJson, assetBaseUrl) {
    const normalizedJson = resolveRobotMeshUrls(robotJson, assetBaseUrl);
    const robot = robotParser(normalizedJson);
    const robotModel = robotCreator(robot);

    robotModel.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });

    this.replaceRobot(robot, robotModel);
    this.setStatus('Roadheader robot loaded');
  }

  replaceRobot(robot, robotModel) {
    if (this.robotModel) {
      this.worldGroup.remove(this.robotModel);
      this.disposeObjectTree(this.robotModel);
    }

    this.destroyRobotGuis();
    this.constraintGuiMap.clear();

    this.robot = robot;
    this.robotModel = robotModel;
    this.worldGroup.add(robotModel);

    this.robotGui = robotGUICreator(robot, robotModel);
    this.workspaceGui = robotWorkSpaceCreator(robot, robotModel);
    this.constraintGui = this.createConstraintGui(robot, robotModel);

    const triangleConstraints = robot.constraints.filter((constraint) => constraint.type === 'triangle-prismatic');
    this.leverPanel.setRobot(robot, robotModel, triangleConstraints, this.constraintGuiMap);
  }

  createConstraintGui(robot, robotModel) {
    const gui = new GUI();
    gui.domElement.style.position = 'absolute';
    gui.domElement.style.left = '0px';
    gui.domElement.style.opacity = '0.8';
    gui.title('Hydraulic Constraints');

    robot.constraints.forEach((constraint) => {
      if (constraint.type !== 'triangle-prismatic') {
        return;
      }

      robotConstraintHandler(robot, robotModel, constraint.name);

      const folder = gui.addFolder(constraint.name);
      const proxy = { length: constraint.length };
      const controller = folder
        .add(proxy, 'length', constraint.limit.lower, constraint.limit.upper)
        .onChange((value) => {
          constraint.length = THREE.MathUtils.clamp(
            value,
            constraint.limit.lower,
            constraint.limit.upper
          );
          robotConstraintHandler(robot, robotModel, constraint.name);
        });

      this.constraintGuiMap.set(constraint.name, { controller, proxy });
    });

    return gui;
  }

  createControlGui() {
    this.controlGui = new GUI();
    this.controlGui.domElement.style.position = 'absolute';
    this.controlGui.domElement.style.left = '0px';
    this.controlGui.domElement.style.opacity = '0.8';
    this.controlGui.domElement.style.top = '450px';
    this.controlGui.title('Controller');

    const folder = this.controlGui.addFolder('Path');
    folder.add({ clearAllKeys: () => this.clearAllKeys() }, 'clearAllKeys');
    folder.add({ saveKeyPosition: () => this.saveKeyPosition() }, 'saveKeyPosition');
    folder.add({ playTrajectory: () => this.playTrajectory() }, 'playTrajectory');
    folder.add({ toggleTarget: () => {
      this.targetMesh.visible = !this.targetMesh.visible;
    } }, 'toggleTarget');
    folder.add({ toggleTransform: () => {
      this.transformControl.visible = !this.transformControl.visible;
      this.targetMesh.visible = this.transformControl.visible || this.targetMesh.visible;
    } }, 'toggleTransform');
  }

  clearAllKeys() {
    this.keyGroup.clear();
    this.trajectoryPositions.fill(0);
    this.lineGeometry.setPositions(this.trajectoryPositions);
    this.trajectoryLineMesh.computeLineDistances();
    this.trajectoryCurve = null;
  }

  saveKeyPosition() {
    const keyMesh = new THREE.Mesh(new THREE.SphereGeometry(0.2, 16, 8), this.targetMaterial);
    keyMesh.position.copy(this.targetMesh.position);
    this.keyGroup.add(keyMesh);

    if (this.keyGroup.children.length > 1) {
      this.updateTrajectoryCurve();
    } else {
      this.trajectoryPositions.fill(0);
      this.lineGeometry.setPositions(this.trajectoryPositions);
      this.trajectoryLineMesh.computeLineDistances();
    }
  }

  updateTrajectoryCurve() {
    this.trajectoryCurve = new THREE.CatmullRomCurve3(
      this.keyGroup.children.map((child) => child.position),
      false,
      'catmullrom',
      0.05
    );

    const point = new THREE.Vector3();
    for (let index = 0; index < ARC_SEGMENTS; index += 1) {
      const t = index / (ARC_SEGMENTS - 1);
      this.trajectoryCurve.getPoint(t, point);
      const base = index * 3;
      this.trajectoryPositions[base] = point.x;
      this.trajectoryPositions[base + 1] = point.y;
      this.trajectoryPositions[base + 2] = point.z;
    }

    this.lineGeometry.setPositions(this.trajectoryPositions);
    this.trajectoryLineMesh.computeLineDistances();
  }

  playTrajectory() {
    if (!this.trajectoryCurve || !this.robot || !this.robotModel) {
      return;
    }

    const speed = 0.02;
    let t = 0;

    const step = () => {
      t += speed / this.trajectoryCurve.getLength();
      if (t >= 1) {
        t = 1;
      }

      const point = this.trajectoryCurve.getPointAt(t);
      this.targetMesh.position.copy(point);
      robotRHIKHandler(this.targetMesh, this.robot, this.robotModel);

      if (t < 1) {
        requestAnimationFrame(step);
      }
    };

    step();
  }

  onResize() {
    if (this.renderer.xr.isPresenting) {
      return;
    }

    this.updateLineResolution();
  }

  updateLineResolution() {
    if (!this.lineMaterial) {
      return;
    }

    const size = new THREE.Vector2();
    this.renderer.getSize(size);
    const dpr = this.renderer.getPixelRatio();
    this.lineMaterial.resolution.set(size.x * dpr, size.y * dpr);
  }

  resizeRendererToDisplaySize() {
    if (this.renderer.xr.isPresenting) {
      return false;
    }

    const pixelRatio = window.devicePixelRatio;
    const width = Math.floor(this.canvas.clientWidth * pixelRatio);
    const height = Math.floor(this.canvas.clientHeight * pixelRatio);
    const needsResize = this.canvas.width !== width || this.canvas.height !== height;

    if (needsResize) {
      this.renderer.setSize(width, height, false);
      this.updateLineResolution();
    }

    return needsResize;
  }

  render() {
    if (this.resizeRendererToDisplaySize()) {
      this.camera.aspect = this.canvas.clientWidth / this.canvas.clientHeight;
      this.camera.updateProjectionMatrix();
    }

    const delta = this.clock.getDelta();
    this.leverPanel.update(delta);
    this.leverPanel.updatePanelToUser();
    this.renderer.render(this.scene, this.camera);
  }

  destroyRobotGuis() {
    [this.robotGui, this.workspaceGui, this.constraintGui].forEach((gui) => {
      if (gui && typeof gui.destroy === 'function') {
        gui.destroy();
      }
    });

    this.robotGui = null;
    this.workspaceGui = null;
    this.constraintGui = null;
  }

  disposeObjectTree(root) {
    root.traverse((object) => {
      if (object.geometry) {
        object.geometry.dispose();
      }
      if (object.material) {
        if (Array.isArray(object.material)) {
          object.material.forEach((material) => material.dispose());
        } else {
          object.material.dispose();
        }
      }
    });
  }
}

import * as THREE from 'three';

import { robotConstraintHandler } from '../robot/robotConstraintHandler.js';

export class LeverPanel {
  constructor({
    renderer,
    panelGroup,
    pointers,
    panelDistance,
    panelYOffset,
    hydraulicSpeed,
    leverDeadzone
  }) {
    this.renderer = renderer;
    this.panelGroup = panelGroup;
    this.pointers = pointers;
    this.panelDistance = panelDistance;
    this.panelYOffset = panelYOffset;
    this.hydraulicSpeed = hydraulicSpeed;
    this.leverDeadzone = leverDeadzone;

    this.robot = null;
    this.robotModel = null;
    this.constraintGuiMap = new Map();
    this.leverItems = [];
    this.activeLeverByPointer = new WeakMap();

    this._box = new THREE.Box3();
    this._center = new THREE.Vector3();
    this._forward = new THREE.Vector3();
    this._panelForward = new THREE.Vector3(0, 0, -1);
  }

  setRobot(robot, robotModel, constraints, constraintGuiMap) {
    this.clear();

    this.robot = robot;
    this.robotModel = robotModel;
    this.constraintGuiMap = constraintGuiMap;
    this.createLeverPanel(constraints);
  }

  clear() {
    this.panelGroup.traverse((object) => {
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

    this.panelGroup.clear();
    this.leverItems = [];
    this.activeLeverByPointer = new WeakMap();
  }

  update(delta) {
    if (!this.robot || !this.robotModel || this.leverItems.length === 0) {
      return;
    }

    this.updateActiveLeversForPointers();
    this.applyLeverPhysicsAndHydraulics(delta);
  }

  updatePanelToUser() {
    if (!this.renderer.xr.isPresenting) {
      return;
    }

    const xrCamera = this.renderer.xr.getCamera();
    this._forward.set(0, 0, -1).applyQuaternion(xrCamera.quaternion);
    this._forward.y = 0;

    if (this._forward.lengthSq() < 1e-6) {
      this._forward.set(0, 0, -1);
    }

    this._forward.normalize();
    this.panelGroup.position.copy(xrCamera.position).addScaledVector(this._forward, this.panelDistance);
    this.panelGroup.position.y = xrCamera.position.y + this.panelYOffset;
    this.panelGroup.quaternion.setFromUnitVectors(this._panelForward, this._forward);
  }

  createLeverPanel(constraints) {
    const panelColor = 0xededed;
    const base = new THREE.Mesh(
      new THREE.BoxGeometry(0.7, 0.12, 0.2),
      new THREE.MeshStandardMaterial({ color: panelColor })
    );

    base.castShadow = true;
    base.receiveShadow = true;
    this.panelGroup.add(base);

    const visibleConstraints = constraints.filter((_, index) => index % 2 === 0);
    const spacing = 0.14;
    const startX = -((visibleConstraints.length - 1) * spacing) / 2;

    visibleConstraints.forEach((constraint, index) => {
      const lever = this.buildLever({
        x: startX + index * spacing,
        z: 0,
        parent: base,
        discLen: 0.1,
        panelColor
      });

      this.leverItems.push({
        pivot: lever.pivot,
        grip: lever.grip,
        minAngle: -Math.PI / 3,
        maxAngle: Math.PI / 3,
        recovery: 3.5,
        constraint,
        lower: constraint.limit.lower,
        upper: constraint.limit.upper
      });
    });
  }

  buildLever({ x = 0, z = 0, parent, discLen = 0.12, panelColor = 0xededed }) {
    const root = new THREE.Group();
    root.position.set(x, 0.06, z);
    parent.add(root);

    const colors = {
      light: 0xb5b5b5,
      dark: 0x1f1f1f,
      rod: 0xbdbdbd,
      handle: 0xf26722
    };

    const makeDisc = (radius, length, color, discX = 0) => {
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, length, 48),
        new THREE.MeshStandardMaterial({ color })
      );
      mesh.rotation.z = Math.PI / 2;
      mesh.position.set(discX, 0, 0);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      return mesh;
    };

    const radius = 0.036;
    root.add(
      makeDisc(radius, 0.03, colors.light, -0.03),
      makeDisc(radius, 0.03, colors.dark, 0),
      makeDisc(radius, 0.03, colors.light, 0.03)
    );

    const cover = new THREE.Mesh(
      new THREE.BoxGeometry(discLen + 0.004, radius + 0.002, radius * 2 + 0.006),
      new THREE.MeshStandardMaterial({ color: panelColor })
    );
    cover.position.set(0, -(radius / 2) + 0.0006, 0);
    root.add(cover);

    const pivot = new THREE.Group();
    pivot.position.set(0, 0.02, 0);
    root.add(pivot);

    const rod = new THREE.Mesh(
      new THREE.CylinderGeometry(0.008, 0.008, 0.28, 24),
      new THREE.MeshStandardMaterial({ color: colors.rod })
    );
    rod.position.y = 0.14;
    rod.castShadow = true;
    pivot.add(rod);

    const handle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.02, 0.012, 0.1, 24),
      new THREE.MeshStandardMaterial({ color: colors.handle })
    );
    handle.position.y = 0.3;
    handle.castShadow = true;
    pivot.add(handle);

    const grip = new THREE.Mesh(
      new THREE.BoxGeometry(0.2, 0.22, 0.2),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.04 })
    );
    grip.visible = false;
    grip.position.y = 0.3;
    pivot.add(grip);

    return { root, pivot, grip };
  }

  updateActiveLeversForPointers() {
    this.pointers.forEach((pointer) => {
      let bestIndex = null;
      let bestDistance = Infinity;

      this.leverItems.forEach((item, index) => {
        if (!this.isPointerInside(pointer, item.grip)) {
          return;
        }

        const center = item.grip.getWorldPosition(this._center);
        const pointerPosition = pointer.getPointerPosition();
        const distance = center.distanceTo(pointerPosition);

        if (distance < bestDistance) {
          bestDistance = distance;
          bestIndex = index;
        }
      });

      if (bestIndex === null) {
        this.activeLeverByPointer.delete(pointer);
      } else {
        this.activeLeverByPointer.set(pointer, bestIndex);
      }
    });
  }

  applyLeverPhysicsAndHydraulics(delta) {
    this.leverItems.forEach((item, leverIndex) => {
      let interacting = false;
      let targetAngle = item.pivot.rotation.x;

      this.pointers.forEach((pointer) => {
        if (this.activeLeverByPointer.get(pointer) !== leverIndex) {
          return;
        }
        if (!this.isPointerInside(pointer, item.grip)) {
          return;
        }

        interacting = true;
        const localPointer = item.pivot.worldToLocal(pointer.getPointerPosition().clone());
        targetAngle = THREE.MathUtils.clamp(
          Math.atan2(localPointer.z, localPointer.y),
          item.minAngle,
          item.maxAngle
        );
      });

      if (interacting) {
        item.pivot.rotation.x = THREE.MathUtils.lerp(item.pivot.rotation.x, targetAngle, 0.5);
      } else {
        item.pivot.rotation.x += -item.pivot.rotation.x * Math.min(1, item.recovery * delta);
      }

      let command = THREE.MathUtils.clamp(
        item.pivot.rotation.x / Math.max(Math.abs(item.minAngle), Math.abs(item.maxAngle)),
        -1,
        1
      );

      if (Math.abs(command) < this.leverDeadzone) {
        command = 0;
      } else {
        const sign = Math.sign(command);
        command = sign * ((Math.abs(command) - this.leverDeadzone) / (1 - this.leverDeadzone));
      }

      const constraint = item.constraint;
      const previousLength = constraint.length;

      if (command !== 0) {
        constraint.length = THREE.MathUtils.clamp(
          constraint.length + command * this.hydraulicSpeed * delta,
          item.lower,
          item.upper
        );
      }

      if (Math.abs(constraint.length - previousLength) > 1e-6) {
        robotConstraintHandler(this.robot, this.robotModel, constraint.name);
        const gui = this.constraintGuiMap.get(constraint.name);
        if (gui) {
          gui.proxy.length = constraint.length;
          gui.controller.updateDisplay();
        }
      }
    });
  }

  isPointerInside(pointer, object) {
    if (!pointer || typeof pointer.getPointerPosition !== 'function') {
      return false;
    }

    if (typeof pointer.intersectBoxObject === 'function') {
      return pointer.intersectBoxObject(object);
    }

    this._box.setFromObject(object);
    return this._box.containsPoint(pointer.getPointerPosition());
  }
}

# Roadheader VR

Independent WebXR project extracted from `robot/demos/roadheader/vr.html`.

## Structure

- `index.html`: application shell and import map fallback.
- `src/`: ES module application code.
- `src/robot/`: robot parser, creator, constraints, GUI and IK helpers copied from the original robot demo.
- `public/assets/roadheader/`: roadheader JSON, GLB meshes and tunnel GLTF used by the VR scene.

## Run

Install dependencies once:

```bash
npm install
```

Start a local dev server:

```bash
npm run dev
```

WebXR requires a secure context. Use `localhost` for desktop testing, or serve with HTTPS when opening from a headset or another device.

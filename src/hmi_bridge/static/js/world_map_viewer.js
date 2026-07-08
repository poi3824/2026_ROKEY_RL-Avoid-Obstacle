import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

const viewerEl = document.getElementById("viewer");
const scanSelect = document.getElementById("scan-select");
const refreshBtn = document.getElementById("refresh-btn");
const statusText = document.getElementById("status-text");
const obstacleTableBody = document.querySelector("#obstacle-table tbody");

// three.js는 Y-up이 기본이지만 base_link(ROS)는 Z-up이다. rosRoot을 X축으로
// -90도 돌려두면, 이 그룹의 자식들은 (x,y,z)를 ROS 좌표 그대로 넣어도 화면에서
// Z가 위로 보인다 - 좌표를 직접 바꾸는 것보다 실수하기 쉬운 부분을 한 곳에 모은다.
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);

const rosRoot = new THREE.Group();
rosRoot.rotation.x = -Math.PI / 2;
scene.add(rosRoot);

rosRoot.add(new THREE.AxesHelper(0.2));
const grid = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
scene.add(grid); // grid는 원래 Y-up 평면이 그대로 base_link의 XY 평면과 맞음

const camera = new THREE.PerspectiveCamera(
  60, viewerEl.clientWidth / viewerEl.clientHeight, 0.01, 100
);
camera.position.set(0.8, -0.8, 0.8);
camera.up.set(0, 0, 1);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(viewerEl.clientWidth, viewerEl.clientHeight);
viewerEl.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.4, 0, 0.1);
controls.update();

window.addEventListener("resize", () => {
  camera.aspect = viewerEl.clientWidth / viewerEl.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(viewerEl.clientWidth, viewerEl.clientHeight);
});

let pointCloudObj = null;
let obstacleGroup = null;

function clearScanObjects() {
  if (pointCloudObj) {
    rosRoot.remove(pointCloudObj);
    pointCloudObj.geometry.dispose();
    pointCloudObj.material.dispose();
    pointCloudObj = null;
  }
  if (obstacleGroup) {
    rosRoot.remove(obstacleGroup);
    obstacleGroup = null;
  }
}

function renderPoints(points) {
  const positions = new Float32Array(points.length * 3);
  let zMin = Infinity;
  let zMax = -Infinity;
  for (let i = 0; i < points.length; i++) {
    positions[i * 3] = points[i][0];
    positions[i * 3 + 1] = points[i][1];
    positions[i * 3 + 2] = points[i][2];
    zMin = Math.min(zMin, points[i][2]);
    zMax = Math.max(zMax, points[i][2]);
  }

  // 높이(z)로 색을 입혀서 테이블(낮음, 파랑)과 장애물 위쪽(높음, 노랑)을 눈으로
  // 구분하기 쉽게 한다 - 이번 세션에서 디버그로 쓰던 matplotlib jet 컬러맵과 같은 의도.
  const colors = new Float32Array(points.length * 3);
  const zSpan = Math.max(zMax - zMin, 1e-6);
  const lowColor = new THREE.Color(0x2255aa);
  const highColor = new THREE.Color(0xffdd33);
  const tmpColor = new THREE.Color();
  for (let i = 0; i < points.length; i++) {
    const t = (points[i][2] - zMin) / zSpan;
    tmpColor.copy(lowColor).lerp(highColor, t);
    colors[i * 3] = tmpColor.r;
    colors[i * 3 + 1] = tmpColor.g;
    colors[i * 3 + 2] = tmpColor.b;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  const material = new THREE.PointsMaterial({ size: 0.004, vertexColors: true });
  pointCloudObj = new THREE.Points(geometry, material);
  rosRoot.add(pointCloudObj);
}

function renderObstacles(obstacles) {
  obstacleGroup = new THREE.Group();

  for (const obs of obstacles) {
    const [cx, cy, cz] = obs.centroid;

    const bodyGeom = new THREE.CylinderGeometry(obs.radius, obs.radius, obs.height, 24, 1, true);
    const bodyMat = new THREE.MeshBasicMaterial({
      color: 0xff9933, wireframe: false, transparent: true, opacity: 0.35, side: THREE.DoubleSide,
    });
    const body = new THREE.Mesh(bodyGeom, bodyMat);
    // CylinderGeometry는 로컬 Y축이 높이 방향이라 90도 돌려서 ROS Z가 높이가 되게 맞춘다.
    body.rotation.x = Math.PI / 2;
    body.position.set(cx, cy, cz);
    obstacleGroup.add(body);

    const safetyGeom = new THREE.CylinderGeometry(
      obs.safety_radius, obs.safety_radius, obs.safety_height, 24, 1, true
    );
    const safetyMat = new THREE.MeshBasicMaterial({
      color: 0xff3333, wireframe: true, transparent: true, opacity: 0.5,
    });
    const safety = new THREE.Mesh(safetyGeom, safetyMat);
    safety.rotation.x = Math.PI / 2;
    safety.position.set(cx, cy, obs.z_min + obs.safety_height / 2);
    obstacleGroup.add(safety);
  }

  rosRoot.add(obstacleGroup);
}

function renderObstacleTable(obstacles) {
  obstacleTableBody.innerHTML = "";
  for (const obs of obstacles.slice().sort((a, b) => a.id - b.id)) {
    const tr = document.createElement("tr");
    const [cx, cy, cz] = obs.centroid;
    tr.innerHTML = `
      <td>${obs.id}</td>
      <td>${cx.toFixed(3)}</td>
      <td>${cy.toFixed(3)}</td>
      <td>${cz.toFixed(3)}</td>
      <td>${(obs.radius * 1000).toFixed(1)}mm</td>
      <td>${(obs.height * 1000).toFixed(1)}mm</td>
      <td>${obs.confidence.toFixed(2)}</td>
    `;
    obstacleTableBody.appendChild(tr);
  }
}

async function fetchJson(url) {
  const res = await fetch(url);
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.error || `${url} -> HTTP ${res.status}`);
  }
  return data;
}

async function loadScan(scanId) {
  statusText.textContent = `로딩 중: ${scanId}`;
  try {
    const [pointsResp, obstaclesResp] = await Promise.all([
      fetchJson(`/api/worldmap/${scanId}/points`),
      fetchJson(`/api/worldmap/${scanId}/obstacles`),
    ]);

    clearScanObjects();
    renderPoints(pointsResp.points);
    renderObstacles(obstaclesResp.obstacles);
    renderObstacleTable(obstaclesResp.obstacles);

    statusText.textContent =
      `${scanId} - ${pointsResp.num_points} points, ${obstaclesResp.obstacles.length} obstacles`;
  } catch (err) {
    statusText.textContent = `에러: ${err.message}`;
  }
}

async function refreshScanList(selectScanId) {
  const listResp = await fetchJson("/api/worldmap/list");
  const scanIds = listResp.scan_ids;

  scanSelect.innerHTML = "";
  for (const id of scanIds) {
    const opt = document.createElement("option");
    opt.value = id;
    opt.textContent = id;
    scanSelect.appendChild(opt);
  }

  if (scanIds.length === 0) {
    statusText.textContent = "저장된 스캔이 없습니다 (data/world_maps 확인)";
    return null;
  }

  const target = selectScanId && scanIds.includes(selectScanId) ? selectScanId : scanIds[0];
  scanSelect.value = target;
  return target;
}

scanSelect.addEventListener("change", () => loadScan(scanSelect.value));
refreshBtn.addEventListener("click", async () => {
  const current = await refreshScanList(scanSelect.value);
  if (current) loadScan(current);
});

(async function init() {
  const initial = await refreshScanList(null);
  if (initial) await loadScan(initial);
})();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

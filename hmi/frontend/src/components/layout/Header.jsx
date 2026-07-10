import { useEffect, useState } from "react";

export default function Header({ title, bridgeConnected }) {
  const [now, setNow] = useState(new Date());

  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(id);
  }, []);

  const pad = (n) => String(n).padStart(2, "0");
  const clock = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())} ${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}`;

  return (
    <div className="topbar">
      <h1>{title}</h1>
      <div className="topbar-right">
        <span className={"badge " + (bridgeConnected ? "good" : "muted")}>
          {bridgeConnected ? "hmi_ros_bridge 연결됨" : "hmi_ros_bridge 연결 안 됨"}
        </span>
        <span className="clock">{clock}</span>
      </div>
    </div>
  );
}

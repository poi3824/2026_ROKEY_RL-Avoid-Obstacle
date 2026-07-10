import Sidebar, { NAV_GROUPS } from "./Sidebar";
import Header from "./Header";
import "./AppShell.css";

function titleFor(activeId) {
  for (const group of NAV_GROUPS) {
    const item = group.items.find((i) => i.id === activeId);
    if (item) return item.label;
  }
  return "";
}

export default function AppShell({ activeId, onSelect, bridgeConnected, children }) {
  return (
    <div className="app">
      <Sidebar activeId={activeId} onSelect={onSelect} />
      <main>
        <Header title={titleFor(activeId)} bridgeConnected={bridgeConnected} />
        <div className="tab-content">{children}</div>
      </main>
    </div>
  );
}

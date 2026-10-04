import { ExtensionContext } from "@foxglove/extension";

import { initOdinDashboard } from "./OdinDashboard";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Odin1 Dashboard", initPanel: initOdinDashboard });
}

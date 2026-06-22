import { ExtensionContext } from "@foxglove/extension";

import { initHorizontalBarPanel } from "./HorizontalBarPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "horizontal-bar",
    initPanel: initHorizontalBarPanel,
  });
}

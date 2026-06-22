import { ExtensionContext } from "@foxglove/extension";

import { initStateTimelinePanel } from "./StateTimelinePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "state-timeline",
    initPanel: initStateTimelinePanel,
  });
}

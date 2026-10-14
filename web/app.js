const elements = {
  status: document.querySelector("#status"),
  description: document.querySelector("#description"),
  board: document.querySelector("#board"),
  filename: document.querySelector("#filename"),
  size: document.querySelector("#size"),
  sha256: document.querySelector("#sha256"),
  sourceCommit: document.querySelector("#source-commit"),
  sdk: document.querySelector("#sdk"),
  eigen: document.querySelector("#eigen"),
  builtAt: document.querySelector("#built-at"),
  download: document.querySelector("#download"),
};

function formatBytes(bytes) {
  return `${new Intl.NumberFormat("ja-JP").format(bytes)} bytes`;
}

async function loadManifest() {
  try {
    const response = await fetch(`./firmware/manifest.json?t=${Date.now()}`, {
      cache: "no-store",
    });
    if (!response.ok) throw new Error(`manifest.json: HTTP ${response.status}`);
    const manifest = await response.json();

    elements.status.textContent = "公開中";
    elements.status.className = "good";
    elements.description.textContent = manifest.description ?? "";
    elements.board.textContent = manifest.boardName ?? manifest.boardId;
    elements.filename.textContent = manifest.downloadName;
    elements.size.textContent = formatBytes(manifest.size);
    elements.sha256.textContent = manifest.sha256;
    elements.sourceCommit.textContent = manifest.sourceCommit;
    elements.sdk.textContent = manifest.picoSdkRef;
    elements.eigen.textContent = manifest.eigenRef;
    elements.builtAt.textContent = new Date(manifest.builtAt).toLocaleString("ja-JP");
    elements.download.href = `./firmware/${manifest.file}?commit=${manifest.sourceCommit}`;
    elements.download.download = manifest.downloadName;
    elements.download.classList.remove("disabled");
    elements.download.removeAttribute("aria-disabled");
  } catch (error) {
    elements.status.textContent = "まだ公開されたUF2がありません";
    elements.status.className = "error";
    elements.description.textContent = error instanceof Error ? error.message : String(error);
  }
}

void loadManifest();

const elements = {
  status: document.querySelector("#status"),
  boardChoice: document.querySelector("#board-choice"),
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

let latestRequestId = 0;

async function loadManifest() {
  const requestId = ++latestRequestId;
  const board = elements.boardChoice.value;
  const manifestFile = board === "pico2" ? "manifest-pico2.json" : "manifest.json";
  elements.download.classList.add("disabled");
  elements.download.setAttribute("aria-disabled", "true");
  elements.download.removeAttribute("href");
  elements.status.textContent = "ビルド情報を取得しています…";
  elements.status.className = "";
  try {
    const response = await fetch(`./firmware/${manifestFile}?t=${Date.now()}`, {
      cache: "no-store",
    });
    if (!response.ok) throw new Error(`${manifestFile}: HTTP ${response.status}`);
    const manifest = await response.json();
    if (requestId !== latestRequestId) return;
    if (manifest.boardId !== board) throw new Error("基板情報が一致しません");

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
    if (requestId !== latestRequestId) return;
    elements.status.textContent = "まだ公開されたUF2がありません";
    elements.status.className = "error";
    elements.description.textContent = error instanceof Error ? error.message : String(error);
  }
}

elements.boardChoice.addEventListener("change", () => { void loadManifest(); });
void loadManifest();

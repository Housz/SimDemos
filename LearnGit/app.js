(() => {
  const pages = ["overview", "save", "branch", "integrate", "remote", "undo"];
  const meta = {
    overview: { n: "01", title: "先看全景，再学命令", intro: "Git 的核心不是命令，而是几个对象之间的状态变化。" },
    save: { n: "02", title: "把修改变成提交", intro: "Working Tree → Staging → Commit，是最常用的日常循环。" },
    branch: { n: "03", title: "在独立开发线上工作", intro: "branch 是指向 commit 的标签；switch 改变你当前所在的开发线。" },
    integrate: { n: "04", title: "把不同开发线重新组合", intro: "merge 保留分叉历史；rebase 会把提交重新接到新的基底。" },
    remote: { n: "05", title: "本地 Git 与 GitHub 协作", intro: "origin、origin/main、fetch、pull、push 和 PR 都在这一层发生。" },
    undo: { n: "06", title: "做错了，回到正确状态", intro: "先判断修改在哪一层，再选择 restore、reset、revert 或 stash。" }
  };

  const commands = [
    { page: "save", cmd: "git status", type: "inspect", desc: "查看当前分支与文件状态。", scene: "status" },
    { page: "save", cmd: "git diff", type: "inspect", desc: "查看尚未 staged 的文本变化。", scene: "diff" },
    { page: "save", cmd: "git add .", type: "save", desc: "把修改加入下一次提交。", scene: "add" },
    { page: "save", cmd: "git commit -m \"...\"", type: "save", desc: "从暂存区创建一个新的项目快照。", scene: "commit" },

    { page: "branch", cmd: "git switch -c feature", type: "branch", desc: "从当前位置创建分支并切过去。", scene: "branchCreate" },
    { page: "branch", cmd: "git switch main", type: "branch", desc: "切换当前分支；工作区随之更新。", scene: "switch" },
    { page: "branch", cmd: "git branch -d feature", type: "branch", desc: "删除分支标签，不等于删除已合并 commit。", scene: "branchDelete" },

    { page: "integrate", cmd: "git merge feature", type: "integrate", desc: "把 feature 合并进当前所在分支。", scene: "merge" },
    { page: "integrate", cmd: "merge conflict", type: "integrate", desc: "两边对同一段内容给出不同结果时，需要人工决定。", scene: "conflict" },
    { page: "integrate", cmd: "git rebase main", type: "advanced", desc: "把当前分支的提交重新接到 main 最新位置后。", scene: "rebase" },

    { page: "remote", cmd: "git fetch", type: "remote", desc: "更新 origin/*，不直接改变当前工作代码。", scene: "fetch" },
    { page: "remote", cmd: "git pull", type: "remote", desc: "获取远程更新，并整合进当前分支。", scene: "pull" },
    { page: "remote", cmd: "git push -u origin feature", type: "remote", desc: "第一次发布新分支并建立跟踪关系。", scene: "push" },
    { page: "remote", cmd: "Pull Request", type: "github", desc: "在 GitHub 上审查、讨论，再把 feature 合入 main。", scene: "pr" },

    { page: "undo", cmd: "git restore file", type: "undo", desc: "丢弃尚未提交的工作区修改。", scene: "restore" },
    { page: "undo", cmd: "git restore --staged file", type: "undo", desc: "把文件移出暂存区，但保留代码修改。", scene: "unstage" },
    { page: "undo", cmd: "git reset --soft HEAD~1", type: "undo", desc: "撤销最近 commit，修改仍保留在 Staging。", scene: "resetSoft" },
    { page: "undo", cmd: "git revert <commit>", type: "undo", desc: "创建一个新 commit，反向撤销已共享的旧提交。", scene: "revert" },
    { page: "undo", cmd: "git stash / git stash pop", type: "pause", desc: "临时收起未完成工作，需要时再恢复。", scene: "stash" }
  ];

  const app = document.querySelector("#app");
  const tabs = [...document.querySelectorAll(".nav-tab")];
  const prev = document.querySelector("#prevPage");
  const next = document.querySelector("#nextPage");
  const pageNumber = document.querySelector("#pageNumber");
  const search = document.querySelector("#commandSearch");
  const results = document.querySelector("#searchResults");
  let current = "overview";

  function graphBase(extra = "") {
    return `<div class="graph">${extra}</div>`;
  }

  function scene(name) {
    const scenes = {
      status: `
        <div class="flow-row">
          <div class="file-stack anim-highlight">
            <span class="lane-label">Working Tree</span>
            <div class="file" data-state="M">solver.cpp</div>
            <div class="file" data-state="+">notes.md</div>
          </div>
          <div class="stage-box anim-highlight"><span class="lane-label">Staging</span><div class="file" style="margin-top:8px">config.json</div></div>
          <div style="min-width:110px" class="anim-highlight"><span class="lane-label">HEAD</span><p style="font:12px var(--mono);margin:10px 0 0">main → C</p></div>
        </div>`,
      diff: `
        <div class="diff anim-highlight">
          <div class="context"> function step(dt) {</div>
          <div class="minus">-  iterations = 4;</div>
          <div class="plus">+  iterations = 8;</div>
          <div class="context"> }</div>
        </div>
        <p class="arrow-text" style="margin-top:18px">Working Tree ↔ 当前已记录版本</p>`,
      add: `
        <div class="flow-row">
          <div class="file-stack"><span class="lane-label">Working Tree</span><div class="file anim-file" data-state="M">solver.cpp</div></div>
          <div><div class="flow-arrow"></div><div class="arrow-text" style="margin-top:8px">git add</div></div>
          <div class="stage-box"><span class="lane-label">Staging</span><div class="file anim-node" style="margin-top:8px">solver.cpp</div></div>
        </div>`,
      commit: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M70 95 H300"/></svg>
        <span class="commit-node" style="left:18%;top:56%"></span>
        <span class="commit-node" style="left:45%;top:56%"></span>
        <span class="commit-node new anim-node" style="left:72%;top:56%"></span>
        <span class="ref anim-ref" style="left:76%;top:39%">main</span>
        <span class="ref head" style="left:76%;top:66%">HEAD</span>
        <span class="ref stage anim-fade" style="left:12%;top:8%">Staging → new commit</span>
      `),
      branchCreate: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M70 90 H260"/></svg>
        <span class="commit-node" style="left:18%;top:53%"></span>
        <span class="commit-node" style="left:58%;top:53%"></span>
        <span class="ref" style="left:62%;top:34%">main</span>
        <span class="ref anim-node" style="left:62%;top:61%">feature</span>
        <span class="ref head anim-head" style="left:62%;top:78%">HEAD</span>
      `),
      switch: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M70 85 H330"/></svg>
        <span class="commit-node" style="left:18%;top:50%"></span>
        <span class="commit-node" style="left:48%;top:50%"></span>
        <span class="commit-node" style="left:78%;top:50%"></span>
        <span class="ref" style="left:52%;top:31%">feature</span>
        <span class="ref" style="left:82%;top:31%">main</span>
        <span class="ref head anim-slide-right" style="left:82%;top:62%">HEAD → main</span>
      `),
      branchDelete: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M70 90 H300"/></svg>
        <span class="commit-node" style="left:18%;top:53%"></span>
        <span class="commit-node" style="left:48%;top:53%"></span>
        <span class="commit-node" style="left:72%;top:53%"></span>
        <span class="ref" style="left:76%;top:33%">main</span>
        <span class="ref anim-fade" style="left:76%;top:62%">feature ×</span>
        <span class="graph-note" style="left:42%;top:78%">commit 仍然存在</span>
      `),
      merge: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true">
          <path class="graph-line" d="M55 85 H145 C190 85 185 42 235 42 H300"/>
          <path class="graph-line branch-line" d="M145 85 C190 85 185 128 235 128 H300 C340 128 340 85 365 85"/>
          <path class="graph-line" d="M300 42 C340 42 340 85 365 85"/>
        </svg>
        <span class="commit-node" style="left:13%;top:50%"></span>
        <span class="commit-node" style="left:35%;top:50%"></span>
        <span class="commit-node" style="left:70%;top:25%"></span>
        <span class="commit-node" style="left:70%;top:75%"></span>
        <span class="commit-node new anim-node" style="left:87%;top:50%"></span>
        <span class="ref" style="left:72%;top:8%">feature</span>
        <span class="ref anim-ref" style="left:87%;top:63%">main + HEAD</span>
      `),
      conflict: `
        <div class="conflict">
          <div class="conflict-choice">main<br><strong>price = 120</strong></div>
          <div class="conflict-bolt anim-conflict">⚡</div>
          <div class="conflict-choice">feature<br><strong>price = 80</strong></div>
          <div class="conflict-output anim-highlight">&lt;&lt;&lt;&lt;&lt;&lt;&lt; HEAD<br>price = 120<br>=======<br>price = 80<br>&gt;&gt;&gt;&gt;&gt;&gt;&gt; feature</div>
        </div>`,
      rebase: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true">
          <path class="graph-line" d="M45 85 H190 H285"/>
          <path class="graph-line branch-line anim-fade" d="M190 85 C220 85 215 40 245 40 H320"/>
          <path class="graph-line branch-line" d="M285 85 H365"/>
        </svg>
        <span class="commit-node" style="left:11%;top:50%"></span>
        <span class="commit-node" style="left:45%;top:50%"></span>
        <span class="commit-node" style="left:68%;top:50%"></span>
        <span class="commit-node new anim-slide-right" style="left:87%;top:50%"></span>
        <span class="ref" style="left:68%;top:63%">main</span>
        <span class="ref anim-slide-right" style="left:86%;top:31%">feature C′ D′</span>
        <span class="graph-note anim-fade" style="left:57%;top:8%">旧 C / D 被重写</span>
      `),
      fetch: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M45 105 H300"/><path class="graph-line remote-line" d="M45 45 H355"/></svg>
        <span class="graph-note" style="left:7%;top:2%">GitHub / origin</span>
        <span class="commit-node" style="left:20%;top:27%"></span><span class="commit-node" style="left:48%;top:27%"></span><span class="commit-node new" style="left:78%;top:27%"></span>
        <span class="graph-note" style="left:7%;top:54%">Local</span>
        <span class="commit-node" style="left:20%;top:62%"></span><span class="commit-node" style="left:48%;top:62%"></span>
        <span class="ref" style="left:49%;top:72%">main</span>
        <span class="ref remote anim-slide-right" style="left:76%;top:43%">origin/main → C</span>
      `),
      pull: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M45 95 H350"/></svg>
        <span class="commit-node" style="left:15%;top:56%"></span><span class="commit-node" style="left:45%;top:56%"></span><span class="commit-node new anim-node" style="left:78%;top:56%"></span>
        <span class="ref anim-slide-right" style="left:79%;top:35%">main</span>
        <span class="ref remote anim-slide-right" style="left:79%;top:67%">origin/main</span>
        <span class="graph-note" style="left:37%;top:8%">fetch + integrate</span>
      `),
      push: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M45 105 H330"/><path class="graph-line remote-line" d="M45 45 H330"/></svg>
        <span class="graph-note" style="left:7%;top:2%">GitHub</span>
        <span class="commit-node" style="left:20%;top:27%"></span><span class="commit-node anim-node" style="left:76%;top:27%"></span>
        <span class="ref remote anim-node" style="left:77%;top:8%">origin/feature</span>
        <span class="graph-note" style="left:7%;top:55%">Local</span>
        <span class="commit-node" style="left:20%;top:62%"></span><span class="commit-node" style="left:76%;top:62%"></span>
        <span class="ref" style="left:77%;top:73%">feature</span>
        <span class="graph-note anim-slide-left" style="left:46%;top:42%">↑ push</span>
      `),
      pr: `
        <div class="pr-flow">
          <div class="pr-box">feature<br><strong>3 commits</strong></div>
          <div class="arrow-text">→</div>
          <div class="pr-box review anim-highlight">Pull Request<br>Diff · Review · CI</div>
          <div class="arrow-text">→</div>
          <div class="pr-box anim-node">main<br><strong>merge</strong></div>
        </div>`,
      restore: `
        <div class="flow-row"><div class="file-stack"><span class="lane-label">Working Tree</span><div class="file anim-fade" data-state="M">solver.cpp</div></div><div class="flow-arrow" style="transform:rotate(180deg)"></div><div><span class="lane-label">HEAD version</span><p style="font:12px var(--mono)">solver.cpp clean</p></div></div>`,
      unstage: `
        <div class="flow-row"><div class="stage-box"><span class="lane-label">Staging</span><div class="file anim-file" style="margin-top:8px">solver.cpp</div></div><div class="flow-arrow"></div><div class="file-stack"><span class="lane-label">Working Tree</span><div class="file anim-node" data-state="M">solver.cpp</div></div></div>`,
      resetSoft: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M60 100 H310"/></svg>
        <span class="commit-node" style="left:20%;top:59%"></span><span class="commit-node anim-fade" style="left:72%;top:59%"></span>
        <span class="ref anim-slide-left" style="left:22%;top:39%">main + HEAD</span>
        <span class="ref stage anim-node" style="left:55%;top:14%">B 的修改 → Staging</span>
      `),
      revert: graphBase(`
        <svg viewBox="0 0 420 170" aria-hidden="true"><path class="graph-line" d="M45 95 H355"/></svg>
        <span class="commit-node" style="left:14%;top:56%"></span><span class="commit-node" style="left:45%;top:56%"></span><span class="commit-node new anim-node" style="left:80%;top:56%"></span>
        <span class="graph-note" style="left:39%;top:30%">B (仍保留)</span>
        <span class="ref anim-ref" style="left:79%;top:36%">C = undo B</span>
      `),
      stash: `
        <div class="flow-row"><div class="file-stack"><span class="lane-label">Working Tree</span><div class="file anim-fade" data-state="M">A.cpp</div><div class="file anim-fade" data-state="M">B.cpp</div></div><div><div class="flow-arrow"></div><div class="arrow-text" style="margin-top:8px">stash ↔ pop</div></div><div class="stage-box anim-node" style="background:var(--panel-2)"><span class="lane-label">Stash</span><p style="font:12px var(--mono);margin:12px 0 0">📦 WIP</p></div></div>`
    };
    return scenes[name] || "";
  }

  function legend() {
    return `<div class="legend">
      <div class="legend-item"><span class="legend-swatch branch"></span>本地 branch</div>
      <div class="legend-item"><span class="legend-swatch stage"></span>Staging</div>
      <div class="legend-item"><span class="legend-swatch remote"></span>remote / origin</div>
      <div class="legend-item"><span class="legend-swatch head"></span>HEAD / 当前状态</div>
    </div>`;
  }

  function pageHead(page) {
    const m = meta[page];
    return `<div class="page-head">
      <div><p class="page-kicker">${m.n} / 06</p><h2>${m.title}</h2><p class="page-intro">${m.intro}</p></div>
      ${legend()}
    </div>`;
  }

  function overview() {
    return `<section class="page" data-page="overview">
      ${pageHead("overview")}
      <div class="world" id="worldMap">
        <div class="world-grid">
          <div class="world-zone zone-work"><h3>WORKING TREE</h3><div class="big-symbol">▤</div><p>你正在编辑的真实文件</p></div>
          <div class="world-arrow"><strong>→</strong>git add</div>
          <div class="world-zone zone-stage"><h3>STAGING</h3><div class="big-symbol">▧</div><p>下一次 commit 的内容</p></div>
          <div class="world-arrow"><strong>→</strong>git commit</div>
          <div class="world-zone zone-local"><h3>LOCAL REPO</h3><div class="big-symbol">●─●</div><p>commit / branch / HEAD</p></div>
          <div class="world-arrow"><strong>⇄</strong>push / fetch</div>
          <div class="world-zone zone-remote"><h3>GITHUB / ORIGIN</h3><div class="big-symbol">☁</div><p>共享的远程 Git 仓库</p></div>
        </div>
        <div class="world-actions"><p>点击播放一次完整的数据流。后面的每张卡片，都只是这张地图的一次局部变化。</p><button class="play" id="playWorld" type="button">播放 →</button></div>
      </div>
      <div class="cards">
        <article class="command-card"><div class="card-head"><code>branch ≠ 代码副本</code><span class="card-type">mental model</span></div><div class="scene">${graphBase(`<svg viewBox="0 0 420 170"><path class="graph-line" d="M60 90 H330"/></svg><span class="commit-node" style="left:18%;top:53%"></span><span class="commit-node" style="left:48%;top:53%"></span><span class="commit-node" style="left:78%;top:53%"></span><span class="ref" style="left:79%;top:34%">main</span><span class="ref head" style="left:79%;top:65%">HEAD</span>`)}</div><div class="card-foot"><p>branch 更像一个会移动的“名字标签”，它指向某个 commit。</p></div></article>
        <article class="command-card"><div class="card-head"><code>origin/main ≠ main</code><span class="card-type">mental model</span></div><div class="scene">${graphBase(`<svg viewBox="0 0 420 170"><path class="graph-line" d="M55 95 H345"/></svg><span class="commit-node" style="left:18%;top:56%"></span><span class="commit-node" style="left:48%;top:56%"></span><span class="commit-node" style="left:78%;top:56%"></span><span class="ref remote" style="left:49%;top:35%">origin/main</span><span class="ref" style="left:79%;top:35%">main</span>`)}</div><div class="card-foot"><p>main 是本地分支；origin/main 是你本地记录的远程 main 位置。</p></div></article>
      </div>
      <div class="callout"><strong>推荐顺序：</strong>先走完 01–05，再看 06 撤销。日常开发主线是：status → diff → add → commit → branch → push → PR → merge。</div>
    </section>`;
  }

  function commandPage(page) {
    const list = commands.filter(c => c.page === page);
    const extra = page === "undo" ? `<div class="callout"><strong>撤销原则：</strong>先判断修改在哪一层。未提交看 restore；刚提交未共享看 reset；已经共享优先 revert。</div>` : "";
    return `<section class="page" data-page="${page}">${pageHead(page)}<div class="cards">${list.map(card).join("")}</div>${extra}</section>`;
  }

  function card(c) {
    return `<article class="command-card" data-command="${escapeAttr(c.cmd)}">
      <div class="card-head"><code>${escapeHTML(c.cmd)}</code><span class="card-type">${c.type}</span></div>
      <div class="scene">${scene(c.scene)}</div>
      <div class="card-foot"><p>${c.desc}</p><button class="play" type="button" aria-label="播放 ${escapeAttr(c.cmd)} 动画">演示 →</button></div>
    </article>`;
  }

  function escapeHTML(s) { return s.replace(/[&<>\"]/g, ch => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[ch])); }
  function escapeAttr(s) { return escapeHTML(s); }

  function attachInteractions() {
    document.querySelectorAll(".command-card[data-command]").forEach(cardEl => {
      const replay = () => {
        cardEl.classList.remove("is-playing");
        void cardEl.offsetWidth;
        cardEl.classList.add("is-playing");
        window.setTimeout(() => cardEl.classList.remove("is-playing"), 1050);
      };
      cardEl.querySelector(".play")?.addEventListener("click", replay);
      cardEl.addEventListener("mouseenter", replay, { passive: true });
    });
    const world = document.querySelector("#worldMap");
    document.querySelector("#playWorld")?.addEventListener("click", () => {
      world.classList.remove("is-playing");
      void world.offsetWidth;
      world.classList.add("is-playing");
      window.setTimeout(() => world.classList.remove("is-playing"), 1600);
    });
  }

  function render(page, targetCommand = "") {
    current = pages.includes(page) ? page : "overview";
    app.innerHTML = current === "overview" ? overview() : commandPage(current);
    tabs.forEach(t => t.classList.toggle("is-active", t.dataset.page === current));
    const i = pages.indexOf(current);
    prev.disabled = i === 0;
    next.disabled = i === pages.length - 1;
    pageNumber.textContent = `${String(i + 1).padStart(2, "0")} / ${String(pages.length).padStart(2, "0")}`;
    attachInteractions();
    if (targetCommand) {
      requestAnimationFrame(() => {
        const cardEl = [...document.querySelectorAll(".command-card[data-command]")].find(el => el.dataset.command === targetCommand);
        if (cardEl) {
          cardEl.classList.add("is-target");
          cardEl.scrollIntoView({ behavior: "smooth", block: "center" });
          window.setTimeout(() => cardEl.classList.remove("is-target"), 1800);
        }
      });
    }
  }

  function navigate(page, targetCommand = "") {
    const hash = `#${page}`;
    if (location.hash !== hash) history.pushState({ page }, "", hash);
    render(page, targetCommand);
    app.focus({ preventScroll: true });
  }

  tabs.forEach(t => t.addEventListener("click", () => navigate(t.dataset.page)));
  prev.addEventListener("click", () => navigate(pages[Math.max(0, pages.indexOf(current) - 1)]));
  next.addEventListener("click", () => navigate(pages[Math.min(pages.length - 1, pages.indexOf(current) + 1)]));
  window.addEventListener("popstate", () => render(location.hash.slice(1) || "overview"));
  window.addEventListener("keydown", e => {
    if (e.target.matches("input, textarea, select")) return;
    if (e.key === "ArrowLeft" && !prev.disabled) navigate(pages[pages.indexOf(current) - 1]);
    if (e.key === "ArrowRight" && !next.disabled) navigate(pages[pages.indexOf(current) + 1]);
  });

  search.addEventListener("input", () => {
    const q = search.value.trim().toLowerCase();
    if (!q) { results.hidden = true; results.innerHTML = ""; return; }
    const found = commands.filter(c => `${c.cmd} ${c.desc}`.toLowerCase().includes(q)).slice(0, 7);
    results.innerHTML = found.length ? found.map(c => `<button class="search-result" type="button" data-page="${c.page}" data-command="${escapeAttr(c.cmd)}"><code>${escapeHTML(c.cmd)}</code><small>${c.desc}</small></button>`).join("") : `<div class="search-result"><small>没有匹配命令</small></div>`;
    results.hidden = false;
    results.querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
      search.value = ""; results.hidden = true; navigate(b.dataset.page, b.dataset.command);
    }));
  });
  document.addEventListener("click", e => {
    if (!e.target.closest(".header-tools")) results.hidden = true;
  });

  render(location.hash.slice(1) || "overview");
})();

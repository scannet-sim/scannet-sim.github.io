// Project page: room tabs, lazy autoplay videos, compare slider, live-demo launcher.
(() => {
  const V = "assets/videos/";

  // ---------------- rooms ----------------
  const ROOMS = [
    { n: 3, name: "Living room + kitchen", objects: 460, movable: 200, joints: 46 },
    { n: 1, name: "Robotics lab", objects: 407, movable: 149, joints: 91 },
    { n: 2, name: "Shared office", objects: 249, movable: 122, joints: 30 },
    { n: 4, name: "Office", objects: 160, movable: 73, joints: 13 },
    { n: 5, name: "Meeting room", objects: 171, movable: 56, joints: 16 },
    { n: 6, name: "Lounge", objects: 80, movable: 42, joints: 6 },
  ];
  const tabs = document.getElementById("room-tabs");
  const meta = document.getElementById("room-meta");
  const vids = { iphone: document.getElementById("v-iphone"), novel: document.getElementById("v-novel"), isaac: document.getElementById("v-isaac") };

  function setRoom(i) {
    const r = ROOMS[i];
    [...tabs.children].forEach((b, k) => b.setAttribute("aria-selected", k === i ? "true" : "false"));
    meta.innerHTML = `<span><b>${r.objects}</b> objects</span><span><b>${r.movable}</b> movable</span><span><b>${r.joints}</b> joints</span>`;
    for (const [k, v] of Object.entries(vids)) {
      v.poster = `${V}room${r.n}_${k}_poster.jpg`;
      v.src = `${V}room${r.n}_${k}.mp4`;
      v.dataset.loaded = "1";
      if (v.dataset.visible === "1") v.play().catch(() => {});
    }
  }
  ROOMS.forEach((r, i) => {
    const b = document.createElement("button");
    b.className = "tab"; b.type = "button"; b.role = "tab"; b.textContent = r.name;
    b.addEventListener("click", () => setRoom(i));
    tabs.appendChild(b);
  });

  // ---------------- lazy autoplay ----------------
  const io = new IntersectionObserver((entries) => {
    for (const e of entries) {
      const v = e.target;
      v.dataset.visible = e.isIntersecting ? "1" : "0";
      if (e.isIntersecting) {
        if (v.id && v.id.startsWith("v-") && !v.dataset.loaded) setRoom(0);
        if (v.preload === "none") v.preload = "auto";
        v.play().catch(() => {});
      } else if (!v.paused) v.pause();
    }
  }, { rootMargin: "200px 0px", threshold: 0.15 });
  document.querySelectorAll("video").forEach((v) => io.observe(v));
  // first room's posters right away, videos when scrolled to
  ROOMS.length && (() => {
    const r = ROOMS[0];
    [...tabs.children].forEach((b, k) => b.setAttribute("aria-selected", k === 0 ? "true" : "false"));
    meta.innerHTML = `<span><b>${r.objects}</b> objects</span><span><b>${r.movable}</b> movable</span><span><b>${r.joints}</b> joints</span>`;
    for (const [k, v] of Object.entries(vids)) v.poster = `${V}room${r.n}_${k}_poster.jpg`;
  })();

  // ---------------- compare slider ----------------
  const D = "assets/detail/";
  const cmp = document.getElementById("cmp");
  const clip = document.getElementById("cmp-clip");
  const handle = document.getElementById("cmp-handle");
  const imgScan = document.getElementById("cmp-scan"), imgOurs = document.getElementById("cmp-ours"), imgPhoto = document.getElementById("cmp-photo");
  const thumbs = document.getElementById("thumbs");
  let pos = 0.5;
  function setPos(p) {
    pos = Math.min(1, Math.max(0, p));
    clip.style.clipPath = `inset(0 0 0 ${pos * 100}%)`;
    handle.style.left = `${pos * 100}%`;
  }
  function pick(it, btn) {
    imgScan.src = `${D}${it.id}_scan.jpg`; imgOurs.src = `${D}${it.id}_ours.jpg`; imgPhoto.src = `${D}${it.id}_photo.jpg`;
    [...thumbs.children].forEach((t) => t.setAttribute("aria-pressed", t === btn ? "true" : "false"));
  }
  fetch(`${D}items.json`).then((r) => r.json()).then((items) => {
    items.forEach((it, i) => {
      const b = document.createElement("button");
      b.className = "thumb"; b.type = "button";
      b.innerHTML = `<img src="${D}${it.id}_thumb.jpg" alt="${it.label}" loading="lazy"><span>${it.label}</span>`;
      b.addEventListener("click", () => pick(it, b));
      thumbs.appendChild(b);
      if (i === 0) pick(it, b);
    });
  });
  let drag = false;
  const at = (e) => { const r = cmp.getBoundingClientRect(); return (e.clientX - r.left) / r.width; };
  cmp.addEventListener("pointerdown", (e) => { drag = true; cmp.setPointerCapture(e.pointerId); setPos(at(e)); });
  cmp.addEventListener("pointermove", (e) => { if (drag) setPos(at(e)); });
  cmp.addEventListener("pointerup", () => { drag = false; });
  cmp.addEventListener("pointercancel", () => { drag = false; });
  const pb = document.getElementById("photo-btn");
  const show = (on) => cmp.classList.toggle("show-photo", on);
  pb.addEventListener("pointerdown", () => show(true));
  ["pointerup", "pointerleave", "pointercancel"].forEach((ev) => pb.addEventListener(ev, () => show(false)));
  pb.addEventListener("keydown", (e) => { if (e.key === " " || e.key === "Enter") show(true); });
  pb.addEventListener("keyup", () => show(false));
  setPos(0.5);

  // ---------------- live demos ----------------
  document.querySelectorAll(".demo-shell").forEach((shell) => {
    const btn = shell.querySelector(".launch");
    if (!btn) return;
    btn.addEventListener("click", () => {
      const src = shell.dataset.src;
      if (!src) return;
      const f = document.createElement("iframe");
      f.src = src; f.allow = "fullscreen; gamepad"; f.title = "Live demo"; f.loading = "eager";
      shell.querySelector(".demo-placeholder").remove();
      shell.appendChild(f);
      const fs = document.createElement("button");
      fs.className = "fs-btn"; fs.type = "button"; fs.textContent = "Full screen";
      fs.addEventListener("click", () => (shell.requestFullscreen ? shell.requestFullscreen() : f.requestFullscreen && f.requestFullscreen()));
      shell.appendChild(fs);
      setTimeout(() => f.focus(), 300);
    });
  });
})();

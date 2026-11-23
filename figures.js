/* LOCH Timer · figuras vectoriales sin rostro.
   Cada ejercicio = 2 poses (A y B) que se interpolan. Las poses se escriben con posiciones
   (x hacia adelante, h = altura sobre el piso) y un solver de 2 huesos calcula codos y rodillas,
   así las extremidades nunca cambian de largo. */
(function (root) {
  'use strict';

  var TOR = 42, UA = 25, FA = 24, TH = 34, SH = 32, FT = 13, NECK = 7, R = 4;
  var BODY = {
    m: { limb: 6.4, torso: 10, sw: 18, hw: 7, sr: 6.4, hr: 5.6, head: 10 },
    f: { limb: 5.6, torso: 8, sw: 13, hw: 10, sr: 5.2, hr: 6.4, head: 9.4 }
  };

  /* ---------- Matemática ---------- */
  function rad(d) { return d * Math.PI / 180; }
  function dirv(t) { return [Math.sin(rad(t)), -Math.cos(rad(t))]; }
  function mix(a, b, s) {
    if (typeof a === 'number' && typeof b === 'number') return a + (b - a) * s;
    if (Array.isArray(a) && Array.isArray(b)) return a.map(function (v, i) { return mix(v, b[i], s); });
    if (a && b && typeof a === 'object') {
      var o = {}, k;
      for (k in a) o[k] = (k in b) ? mix(a[k], b[k], s) : a[k];
      for (k in b) if (!(k in a)) o[k] = b[k];
      return o;
    }
    return a === undefined ? b : a;
  }
  function P() {
    var o = {};
    for (var i = 0; i < arguments.length; i++) { var a = arguments[i]; for (var k in a) if (a[k] !== undefined) o[k] = a[k]; }
    return o;
  }
  // Solver de 2 huesos. hint = hacia dónde "apunta" el codo/rodilla.
  function ik(r, t, l1, l2, hint) {
    var dx = t[0] - r[0], dy = t[1] - r[1], d = Math.hypot(dx, dy), short = 0;
    if (d < 0.001) { dx = 0; dy = -0.001; d = 0.001; }
    var max = l1 + l2 - 0.02, min = Math.abs(l1 - l2) + 0.5;
    if (d > max) { short = d - max; var k = max / d; dx *= k; dy *= k; d = max; }
    else if (d < min) { var k2 = min / d; dx *= k2; dy *= k2; d = min; }
    var a = (l1 * l1 - l2 * l2 + d * d) / (2 * d);
    var h = Math.sqrt(Math.max(0, l1 * l1 - a * a));
    var ux = dx / d, uy = dy / d, px = -uy, py = ux;
    var side = (px * hint[0] + py * hint[1]) >= 0 ? 1 : -1;
    return { mid: [r[0] + ux * a + px * h * side, r[1] + uy * a + py * h * side], end: [r[0] + dx, r[1] + dy], short: short };
  }

  /* ---------- Esqueleto ---------- */
  function solve(Pz, view, sex) {
    var B = BODY[sex], front = view === 'front';
    var hip = Pz.hip, sh;
    if (Pz.sh) {
      var dx = Pz.sh[0] - hip[0], dy = Pz.sh[1] - hip[1], d = Math.hypot(dx, dy) || 1;
      sh = [hip[0] + dx / d * TOR, hip[1] + dy / d * TOR];
    } else { var v = dirv(Pz.t); sh = [hip[0] + v[0] * TOR, hip[1] + v[1] * TOR]; }
    var tdeg = Math.atan2(sh[0] - hip[0], -(sh[1] - hip[1])) * 180 / Math.PI;
    var hu = dirv(tdeg + (Pz.hd || 0));
    var hc = [sh[0] + hu[0] * (NECK + B.head), sh[1] + hu[1] * (NECK + B.head)];
    var aN = front ? [sh[0] + B.sw, sh[1]] : sh, aF = front ? [sh[0] - B.sw, sh[1]] : sh;
    var lN = front ? [hip[0] + B.hw, hip[1]] : hip, lF = front ? [hip[0] - B.hw, hip[1]] : hip;
    function mir(v) { return [-v[0], v[1]]; }
    function rel(r, o) { return [r[0] + o[0], r[1] + o[1]]; }
    var ekN = Pz.ek || [-1, 0], kkN = Pz.kk || [1, 0];
    var hN = Pz.hr ? rel(aN, Pz.hr) : Pz.hand;
    var hF = Pz.hr2 ? rel(aF, Pz.hr2) : Pz.hand2 ? Pz.hand2 : front ? (Pz.hr ? rel(aF, mir(Pz.hr)) : mir(Pz.hand)) : hN;
    var fN = Pz.foot, fF = Pz.foot2 || (front ? mir(Pz.foot) : Pz.foot);
    var ekF = Pz.ek2 || (front ? mir(ekN) : ekN), kkF = Pz.kk2 || (front ? mir(kkN) : kkN);
    var fdN = Pz.fd === undefined ? 90 : Pz.fd;
    var fdF = Pz.fd2 !== undefined ? Pz.fd2 : (front ? -fdN : fdN);
    var a1 = ik(aN, hN, UA, FA, ekN), a2 = ik(aF, hF, UA, FA, ekF);
    var l1 = ik(lN, fN, TH, SH, kkN), l2 = ik(lF, fF, TH, SH, kkF);
    function toe(an, fd) { var f = dirv(fd); return [an[0] + f[0] * FT, an[1] + f[1] * FT]; }
    return {
      view: view, B: B, hip: hip, sh: sh, head: hc, hu: hu, tdeg: tdeg,
      armN: [aN, a1.mid, a1.end], armF: [aF, a2.mid, a2.end],
      legN: [lN, l1.mid, l1.end, toe(l1.end, fdN)], legF: [lF, l2.mid, l2.end, toe(l2.end, fdF)],
      short: Math.max(a1.short, a2.short, l1.short, l2.short)
    };
  }

  /* ---------- Dibujo ---------- */
  function n1(v) { return Math.round(v * 10) / 10; }
  function pt(p) { return n1(p[0]) + ',' + n1(-p[1]); }
  function pl(pts, w, op) {
    return '<polyline points="' + pts.map(pt).join(' ') + '" fill="none" stroke="currentColor" stroke-width="' + w +
      '" stroke-linecap="round" stroke-linejoin="round"' + (op < 1 ? ' opacity="' + op + '"' : '') + '/>';
  }
  function circle(c, r, extra) { return '<circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="' + r + '" fill="currentColor"' + (extra || '') + '/>'; }
  var OUT = 'stroke="var(--bgc,#fff)" stroke-width="1.6"';

  function dumb(c, type) {
    if (type === 'o') return circle(c, 6.4, ' ' + OUT);
    if (type === 'g') return circle(c, 8.6, ' ' + OUT) + '<circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="3.6" fill="var(--acc,#fff)"/>';
    var vert = type === 'v', ax = vert ? 0 : 9, ay = vert ? 9 : 0, px = vert ? 6 : 0, py = vert ? 0 : 6;
    var e1 = [c[0] - ax, c[1] - ay], e2 = [c[0] + ax, c[1] + ay], s = '';
    function ln(a, b, w, col) { return '<line x1="' + n1(a[0]) + '" y1="' + n1(-a[1]) + '" x2="' + n1(b[0]) + '" y2="' + n1(-b[1]) + '" stroke="' + col + '" stroke-width="' + w + '" stroke-linecap="round"/>'; }
    var bg = 'var(--bgc,#fff)', fg = 'currentColor';
    s += ln(e1, e2, 6.2, bg) + ln([e1[0] - px, e1[1] - py], [e1[0] + px, e1[1] + py], 8, bg) + ln([e2[0] - px, e2[1] - py], [e2[0] + px, e2[1] + py], 8, bg);
    s += ln(e1, e2, 3, fg) + ln([e1[0] - px, e1[1] - py], [e1[0] + px, e1[1] + py], 4.6, fg) + ln([e2[0] - px, e2[1] - py], [e2[0] + px, e2[1] + py], 4.6, fg);
    return s;
  }

  function chair(x0, x1, h, back) {
    var s = '<g stroke="currentColor" stroke-opacity=".42" stroke-width="3.6" stroke-linecap="round" fill="none">';
    s += '<line x1="' + x0 + '" y1="' + -h + '" x2="' + x1 + '" y2="' + -h + '"/>';
    s += '<line x1="' + (x0 + 3) + '" y1="' + -h + '" x2="' + (x0 + 3) + '" y2="-1"/>';
    s += '<line x1="' + (x1 - 3) + '" y1="' + -h + '" x2="' + (x1 - 3) + '" y2="-1"/>';
    if (back) s += '<line x1="' + back[0] + '" y1="' + -h + '" x2="' + back[0] + '" y2="' + -back[1] + '"/>';
    return s + '</g>';
  }


  function faceSvg(J, expr, face, front) {
    if (!expr) return '';
    var B = J.B, r = B.head, u = J.hu, f = front ? [1, 0] : [u[1] * (face || 1), -u[0] * (face || 1)];
    var BGc = 'var(--bgc,#fff)', o = '';
    function at(a, b) { return [J.head[0] + f[0] * a * r + u[0] * b * r, J.head[1] + f[1] * a * r + u[1] * b * r]; }
    function eye(c, kind, side) {
      var x = c[0], y = -c[1], rot = -Math.atan2(u[0], u[1]) * 180 / Math.PI, k = front ? -(side || 1) : 1;
      var g = '<g transform="rotate(' + n1(rot) + ' ' + n1(x) + ' ' + n1(y) + ')">';
      if (kind === 'open') g += '<path d="M' + n1(x - 3.2) + ' ' + n1(y - 0.9) + ' H' + n1(x + 3.2) + ' A3.2 2.7 0 0 1 ' + n1(x - 3.2) + ' ' + n1(y - 0.9) + ' Z" fill="' + BGc + '"/>';
      else if (kind === 'angry') g += '<ellipse cx="' + n1(x) + '" cy="' + n1(y) + '" rx="3.2" ry="2.3" fill="' + BGc + '"/><path d="M' + n1(x - 4.4 * k) + ' ' + n1(y - 3.8) + ' L' + n1(x + 4.4 * k) + ' ' + n1(y - 3.8) + ' L' + n1(x + 4.4 * k) + ' ' + n1(y + 0.4) + ' Z" fill="currentColor"/>';
      else if (kind === 'happy') g += '<path d="M' + n1(x - 3.1) + ' ' + n1(y + 1.4) + ' Q' + n1(x) + ' ' + n1(y - 3) + ' ' + n1(x + 3.1) + ' ' + n1(y + 1.4) + '" fill="none" stroke="' + BGc + '" stroke-width="1.9" stroke-linecap="round"/>';
      else if (kind === 'squint') g += '<path d="M' + n1(x - 3 * k) + ' ' + n1(y - 2.4) + ' L' + n1(x + 2 * k) + ' ' + n1(y) + ' L' + n1(x - 3 * k) + ' ' + n1(y + 2.4) + '" fill="none" stroke="' + BGc + '" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"/>';
      return g + '</g>';
    }
    var ek = { neutral: 'open', effort: 'angry', smile: 'happy', shout: 'squint' }[expr] || 'open';
    var hi = (expr === 'smile' || expr === 'shout') ? -0.18 : -0.28;
    if (front) o += eye(at(-0.42, hi), ek, -1) + eye(at(0.42, hi), ek, 1);
    else o += eye(at(0.4, hi - 0.02), ek, 1);
    var m = at(front ? 0 : 0.36, -0.66), mx = m[0], my = -m[1];
    if (expr === 'effort') o += '<path d="M' + n1(mx - 2.6) + ' ' + n1(my) + ' L' + n1(mx + 2.6) + ' ' + n1(my) + '" stroke="' + BGc + '" stroke-width="1.7" stroke-linecap="round"/>';
    else if (expr === 'smile') o += '<path d="M' + n1(mx - 3) + ' ' + n1(my - 1) + ' H' + n1(mx + 3) + ' A3 2.7 0 0 1 ' + n1(mx - 3) + ' ' + n1(my - 1) + ' Z" fill="' + BGc + '"/>';
    else if (expr === 'shout') o += '<ellipse cx="' + n1(mx) + '" cy="' + n1(my + 0.4) + '" rx="2.5" ry="2.9" fill="' + BGc + '"/>';
    return o;
  }
  /* ---- Zona trabajada: se ilumina sobre el muñeco ---- */
  var MUS = { arms: ['shoulder'], jacks: ['full'], march: ['quad', 'core'], hamfold: ['ham'], hipflex: ['quad'], tristr: ['tri'], calfstr: ['calf'], breath: ['core'],
    pushup: ['chest', 'tri'], chestpress: ['chest', 'tri'], flies: ['chest'], dips: ['tri', 'chest'], triext: ['tri'], goblet: ['quad', 'glute'], lunge: ['quad', 'glute'],
    rdl: ['glute', 'ham'], bridge: ['glute'], calf: ['calf'], sidekick: ['glute', 'leg'], situp: ['core'], wheel: ['core'], legraise: ['core'], bicycle: ['core'],
    sideplank: ['core'], plank: ['core'], row1: ['back', 'bi'], row2: ['back', 'bi'], superman: ['back', 'glute'], curl: ['bi'], hammer: ['bi', 'grip'],
    shpress: ['shoulder', 'tri'], latraise: ['shoulder'], frontraise: ['shoulder'], rope: ['calf', 'shoulder'], squatjump: ['quad', 'glute', 'calf'],
    climbers: ['core', 'shoulder'], jabcross: ['shoulder', 'arm'], hook: ['shoulder', 'core'], uppercut: ['shoulder', 'arm', 'core'], slip: ['core'], boxstep: ['calf'],
    airsquat: ['quad', 'glute'], burpee: ['full'], deadbug: ['core'], pullup: ['back', 'bi'], wallsit: ['quad'], grip: ['grip'] };
  var MUSN = { chest: 'Pecho', back: 'Espalda', shoulder: 'Hombros', bi: 'Bíceps', tri: 'Tríceps', arm: 'Brazos', core: 'Abdomen', glute: 'Glúteos', quad: 'Cuádriceps',
    ham: 'Isquios', calf: 'Pantorrillas', grip: 'Antebrazo y agarre', full: 'Cuerpo completo', leg: 'Piernas' };
  function muscles(id) { return (MUS[id] || []).map(function (k) { return MUSN[k]; }); }
  function hlLayer(J, tags, p, front) {
    var B = J.B, o = '', W = B.limb * 1.02;
    function seg(a, b, w) { return '<line x1="' + n1(a[0]) + '" y1="' + n1(-a[1]) + '" x2="' + n1(b[0]) + '" y2="' + n1(-b[1]) + '" stroke-width="' + n1(w) + '" stroke-linecap="round"/>'; }
    function dot(c, r) { return '<circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="' + n1(r) + '" stroke="none"/>'; }
    function lerp(a, b, f) { return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]; }
    var tw = front ? B.sw * 1.3 : B.torso * 0.6, arms = [J.armN, J.armF], legs = [J.legN, J.legF];
    tags.forEach(function (t) {
      if (t === 'full') { o += seg(J.hip, J.sh, tw); arms.concat(legs).forEach(function (c) { o += seg(c[0], c[1], W) + seg(c[1], c[2], W); }); }
      else if (t === 'chest' || t === 'back') o += seg(lerp(J.sh, J.hip, 0.05), lerp(J.sh, J.hip, 0.5), tw);
      else if (t === 'core') o += seg(lerp(J.sh, J.hip, 0.5), lerp(J.sh, J.hip, 0.95), tw);
      else if (t === 'shoulder') { o += dot(J.sh, B.limb * 1.05); if (front) { o += dot([J.sh[0] + B.sw, J.sh[1]], B.limb * 1.05) + dot([J.sh[0] - B.sw, J.sh[1]], B.limb * 1.05); } }
      else if (t === 'bi' || t === 'tri') arms.forEach(function (c) { o += seg(c[0], c[1], W); });
      else if (t === 'arm') arms.forEach(function (c) { o += seg(c[0], c[1], W) + seg(c[1], c[2], W); });
      else if (t === 'grip') arms.forEach(function (c) { o += seg(c[1], c[2], W) + dot(c[2], B.limb * 0.9); });
      else if (t === 'glute') o += dot(J.hip, B.hr * 1.15);
      else if (t === 'quad' || t === 'ham') legs.forEach(function (c) { o += seg(c[0], c[1], W); });
      else if (t === 'calf') legs.forEach(function (c) { o += seg(c[1], c[2], W); });
      else if (t === 'leg') legs.forEach(function (c) { o += seg(c[0], c[1], W) + seg(c[1], c[2], W); });
    });
    var op = 0.86 + 0.14 * Math.sin(2 * Math.PI * p * 2);
    return '<g stroke="var(--hl,#ffd60a)" fill="var(--hl,#ffd60a)" opacity="' + n1(op * 100) / 100 + '" style="mix-blend-mode:normal">' + o + '</g>';
  }
  function drawFigure(J, ex, sex, p, s, expr, hl) {
    var B = J.B, front = J.view === 'front', out = '';
    var u = J.hu, face = ex.face || 1, f = [u[1] * face, -u[0] * face], back = [-f[0], -f[1]];
    var op = front ? 1 : 0.5;
    if (ex.bg) out += ex.bg(J, p, s);
    function limb(c, w, o) {
      var e = c[c.length - 1], t = '';
      t += pl(c, w, o);
      if (c.length === 3) t += '<circle cx="' + n1(e[0]) + '" cy="' + n1(-e[1]) + '" r="' + n1(w * 0.78) + '" fill="currentColor"' + (o < 1 ? ' opacity="' + o + '"' : '') + '/>';
      return t;
    }
    // lejanas
    out += limb(J.legF, B.limb, op) + limb(J.armF, B.limb, op);
    if (ex.db && !ex.dbN) out += '<g opacity="' + (front ? 1 : 0.6) + '">' + dumb(J.armF[2], ex.db) + '</g>';
    // tronco
    if (front) {
      var sw = B.sw, hw = B.hw, S = J.sh, H = J.hip;
      out += '<path d="M' + pt([S[0] - sw + 2, S[1]]) + ' L' + pt([S[0] + sw - 2, S[1]]) + ' L' + pt([H[0] + hw - 1, H[1]]) + ' L' + pt([H[0] - hw + 1, H[1]]) + ' Z" fill="currentColor" stroke="currentColor" stroke-width="7" stroke-linejoin="round"/>';
    } else {
      out += pl([J.hip, J.sh], B.torso * 0.8, 1);
      var ch = [J.sh[0] + (J.hip[0] - J.sh[0]) * 0.22, J.sh[1] + (J.hip[1] - J.sh[1]) * 0.22];
      out += circle(ch, B.torso * 0.72) + circle(J.hip, B.hr * 0.9);
    }
    // cuello y cabeza (estilo stickman: cabeza grande, bufanda con colas al viento)
    var nk = [J.sh[0] + u[0] * 8, J.sh[1] + u[1] * 8];
    out += pl([J.sh, nk], B.limb * 0.9, 1);
    out += circle(J.head, B.head);
    var BG = 'var(--bgc,#fff)', hr_ = B.head;
    var tx = [-u[1], u[0]]; // perpendicular a "arriba de la cabeza"
    var c0 = [J.head[0] + u[0] * 1.5, J.head[1] + u[1] * 1.5];
    // banda: hueco del color del fondo sobre la cabeza
    var bw = Math.sqrt(Math.max(hr_ * hr_ - 2.25, 4)) - 0.6;
    out += '<line x1="' + n1(c0[0] - tx[0] * bw) + '" y1="' + n1(-(c0[1] - tx[1] * bw)) + '" x2="' + n1(c0[0] + tx[0] * bw) + '" y2="' + n1(-(c0[1] + tx[1] * bw)) + '" stroke="var(--acc,' + BG + ')" stroke-width="3" stroke-linecap="butt"/>';
    var hv = J.hv || [0, 0];
    var xp = expr || ex.expr; out += faceSvg(J, typeof xp === 'function' ? xp(p) : xp, ex.face, front);
    if (!front) {
      var fl = Math.sin(2 * Math.PI * (p * 2)) * 2.2;
      var dx = J.head[0] - J.hip[0], dr = Math.abs(dx) > 6 ? (dx > 0 ? -1 : 1) : -(face || 1);
      var e0 = [J.head[0] + dr * (hr_ - 1), J.head[1] + 2];
      var lx = -hv[0] * 0.5, ly = -hv[1] * 0.5 - 1.5; lx = Math.max(-14, Math.min(14, lx)); ly = Math.max(-12, Math.min(12, ly));
      var q1 = [e0[0] + dr * 11 + lx * 0.5, e0[1] + 1.5 + fl * 0.6 + ly * 0.5], q2 = [e0[0] + dr * 21 + lx, e0[1] - 3 - fl * 0.6 + ly];
      out += '<path d="M' + pt(e0) + ' Q' + pt(q1) + ' ' + pt(q2) + '" fill="none" stroke="var(--acc,currentColor)" stroke-width="3.6" stroke-linecap="round"/>';
      var q3 = [e0[0] + dr * 10 + lx * 0.4, e0[1] - 3.5 - fl * 0.6 + ly * 0.4], q4 = [e0[0] + dr * 18 + lx * 0.9, e0[1] - 9 + fl * 0.6 + ly * 0.8];
      out += '<path d="M' + pt(e0) + ' Q' + pt(q3) + ' ' + pt(q4) + '" fill="none" stroke="var(--acc,currentColor)" stroke-width="2.8" stroke-linecap="round"/>';
    }
    if (sex === 'f') {
      if (front) {
        out += circle([J.head[0] + u[0] * (B.head + 1), J.head[1] + u[1] * (B.head + 1)], 4);
      } else {
        var gv = [back[0] * 0.5, back[1] * 0.5 - 1], gl = Math.hypot(gv[0], gv[1]) || 1;
        var b0 = [J.head[0] + back[0] * B.head * 0.6 + u[0] * 3, J.head[1] + back[1] * B.head * 0.6 + u[1] * 3];
        var b1 = [b0[0] + gv[0] / gl * 12, b0[1] + gv[1] / gl * 12];
        if (b1[1] < 3) b1[1] = 3;
        out += pl([b0, b1], 5, 1);
      }
    }
    // cinturón (corte del color del fondo en la cadera)
    if (!front) {
      var hp = J.hip, tv = [J.sh[0] - J.hip[0], J.sh[1] - J.hip[1]], tl = Math.hypot(tv[0], tv[1]) || 1;
      var tu = [tv[0] / tl, tv[1] / tl], pn = [-tu[1], tu[0]], bp = [hp[0] + tu[0] * 4, hp[1] + tu[1] * 4], hw2 = B.torso / 2 + 0.5;
      out += '<line x1="' + n1(bp[0] - pn[0] * hw2) + '" y1="' + n1(-(bp[1] - pn[1] * hw2)) + '" x2="' + n1(bp[0] + pn[0] * hw2) + '" y2="' + n1(-(bp[1] + pn[1] * hw2)) + '" stroke="' + BG + '" stroke-width="1.8"/>';
    }
    // cercanas
    out += limb(J.legN, B.limb, 1) + limb(J.armN, B.limb, 1);
    if (hl && MUS[ex.id]) out += hlLayer(J, MUS[ex.id], p, front);
    if (ex.db) out += dumb(J.armN[2], ex.db);
    if (ex.fg) out += ex.fg(J, p, s);
    return out;
  }

  /* ---------- Poses ---------- */
  var STAND = { hip: [0, 66], t: 180, hd: 0, hand: [4, 60], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] };
  var SUP = { hip: [0, R], sh: [-42, R], hd: -15, hand: [5, R], foot: [42, R], fd: 90, ek: [0, -1], kk: [0, 1] };
  var LEN = TOR + TH + SH;
  function lineSh(shH, an) { // cuerpo recto: calcula x del hombro dado el tobillo
    var sx = an[0] + Math.sqrt(LEN * LEN - Math.pow(shH - an[1], 2)), k = TOR / LEN, S = [sx, shH];
    return { sh: S, hip: [S[0] + (an[0] - S[0]) * k, S[1] + (an[1] - S[1]) * k], foot: an };
  }
  function lineAnk(S, ah) { // cuerpo recto: calcula x del tobillo dado el hombro
    var ax = S[0] - Math.sqrt(LEN * LEN - Math.pow(S[1] - ah, 2)), an = [ax, ah], k = TOR / LEN;
    return { sh: S, hip: [S[0] + (an[0] - S[0]) * k, S[1] + (an[1] - S[1]) * k], foot: an };
  }
  var HINGE = { hip: [-26, 60], t: 105, hd: 10, foot: [0, R], fd: 90, ek: [-1, 0.3], kk: [1, 0.2] };

  var pA = [-66, 15], pu0 = lineSh(51, pA), pu1 = lineSh(22, pA);
  var PLK = { hd: -12, fd: 20, ek: [-1, -1], kk: [0, 1] };
  var sp0 = lineAnk([30, 29], 8), sp1 = lineAnk([30, 29], 8);
  var fp0 = lineAnk([27, 29], 15), fp1 = lineAnk([27, 30], 15);
  var CL = { hip: [-4.6, 39.1], t: 105, hand: [36, R], hd: -12, fd: 20, ek: [-1, 0], kk: [1, 1], kk2: [1, 1] };

  function shOf(h, t) { var v = dirv(t); return [h[0] + v[0] * TOR, h[1] + v[1] * TOR]; }
  var BX = { hip: [0, 62], t: 175, hd: 0, foot: [14, R], foot2: [-16, R], fd: 90, fd2: 90, kk: [1, 0.3], kk2: [1, 0.3], ek: [0, -1], ek2: [0, -1], hr: [16, 2], hr2: [16, 2] };
  var BXF = P(STAND, { hip: [0, 62], foot: [14, R], hr: [-8, 0], hr2: [8, 0], ek: [1, -1], ek2: [-1, -1] });
  function bump(x) { return Math.sin(Math.PI * Math.max(0, Math.min(1, x))); }
  function keys(K, p) { var n = K.length - 1, x = p * n, i = Math.min(n - 1, Math.floor(x)), f = x - i; return mix(K[i], K[i + 1], 0.5 - 0.5 * Math.cos(Math.PI * f)); }
  var BU_S0 = { hip: [0, 66], sh: [0, 108], hd: 0, hand: [4, 60], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] };
  var BU_S1 = { hip: [-30, 34], sh: shOf([-30, 34], 115), hd: 10, hand: [18, 6], foot: [-6, R], fd: 90, ek: [0, 1], kk: [1, 0.3] };
  var BU_S2 = { hip: pu0.hip, sh: pu0.sh, hd: -12, hand: [pu0.sh[0], R], foot: pu0.foot, fd: 20, ek: [-1, -1], kk: [0, 1] };
  var BU_S3 = { hip: [0, 92], sh: [0, 134], hd: 0, hand: [3, 180], foot: [0, 26], fd: 140, ek: [-1, 0], kk: [1, 0.2] };
  var BURPEE = [BU_S0, BU_S1, BU_S2, BU_S2, BU_S1, BU_S3, BU_S0];

  function keysT(K, p) { var i = 0; while (i < K.length - 2 && p > K[i + 1].at) i++; var a = K[i], b = K[i + 1], f = Math.max(0, Math.min(1, (p - a.at) / (b.at - a.at))); f = 0.5 - 0.5 * Math.cos(Math.PI * f); return mix(a.k, b.k, f); }
  /* ---- Motor de movimiento: curvas con anticipación y rebote ---- */
  function cl01(x) { return Math.max(0, Math.min(1, x)); }
  function eIO(x) { x = cl01(x); return 0.5 - 0.5 * Math.cos(Math.PI * x); }
  function eOut(x) { x = cl01(x); return 1 - (1 - x) * (1 - x) * (1 - x); }
  function eBack(x, k) { x = cl01(x); k = k || 1.4; var c = x - 1; return 1 + (k + 1) * c * c * c + k * c * c; }   // pasa de 1 y vuelve
  // golpe: 0 reposo -> -0.14 (carga) -> 1.08 (impacto, rebote) -> 1 (extendido) -> 0 (regreso rápido)
  function punch(q) {
    if (q < 0.16) return -0.14 * eIO(q / 0.16);
    if (q < 0.34) return -0.14 + 1.14 * eBack((q - 0.16) / 0.18, 1.6);
    if (q < 0.44) return 1;
    return Math.max(0, 1 - eIO((q - 0.44) / 0.5)) * (q < 0.94 ? 1 : 0);
  }
  var BU_S1b = { hip: [-34, 27], sh: shOf([-34, 27], 120), hd: 12, hand: [14, 6], foot: [-8, R], fd: 90, ek: [0, 1], kk: [1, 0.3] };
  var BU_L = { hip: [-6, 44], sh: shOf([-6, 44], 160), hd: 0, hand: [-18, 46], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.4] };
  var BU_S2b = { hip: [pu0.hip[0], pu0.hip[1] + 5], sh: pu0.sh, hd: -12, hand: [pu0.sh[0], R], foot: pu0.foot, fd: 20, ek: [-1, -1], kk: [0, 1] };
  var BURPEE_T = [
    { at: 0.00, k: BU_S0 }, { at: 0.10, k: BU_S0 }, { at: 0.20, k: BU_S1 }, { at: 0.29, k: BU_S2b }, { at: 0.34, k: BU_S2 },
    { at: 0.40, k: BU_S2 }, { at: 0.50, k: BU_S1b }, { at: 0.58, k: BU_S1 }, { at: 0.70, k: BU_S3 }, { at: 0.80, k: BU_L }, { at: 0.90, k: BU_S0 }, { at: 1.00, k: BU_S0 }
  ];
  function pushS(p) {   // 1 = abajo. Baja lento, pausa, sube más rápido con rebote, respira arriba
    if (p < 0.46) return eIO(p / 0.46);
    if (p < 0.54) return 1;
    if (p < 0.86) return 1 - eBack((p - 0.54) / 0.32, 1.1);
    return 0;
  }

  var VK = [
    { at: 0.00, k: { hip: [0, 66], sh: [0, 108], hd: 0, hand: [4, 60], hand2: [-4, 60], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] } },
    { at: 0.14, k: { hip: [-6, 46], sh: shOf([-6, 46], 160), hd: 0, hand: [-24, 48], hand2: [-24, 46], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.4] } },
    { at: 0.30, k: { hip: [0, 114], sh: [0, 156], hd: 0, hand: [16, 202], hand2: [-12, 200], foot: [0, 52], fd: 140, ek: [-1, 0], kk: [1, 0.2] } },
    { at: 0.50, k: { hip: [0, 104], sh: [0, 146], hd: -4, hand: [18, 192], hand2: [-12, 190], foot: [4, 40], fd: 130, ek: [-1, 0], kk: [1, 0.2] } },
    { at: 0.66, k: { hip: [0, 56], sh: shOf([0, 56], 176), hd: 0, hand: [22, 92], hand2: [14, 96], foot: [0, R], fd: 90, ek: [0, -1], kk: [1, 0.4] } },
    { at: 0.82, k: { hip: [0, 66], sh: [0, 108], hd: -4, hand: [18, 150], hand2: [-2, 60], foot: [0, R], fd: 90, ek: [0, -1], kk: [1, 0.2] } },
    { at: 1.00, k: { hip: [0, 66], sh: [0, 108], hd: -4, hand: [18, 152], hand2: [-2, 60], foot: [0, R], fd: 90, ek: [0, -1], kk: [1, 0.2] } }
  ];
  var VSPLIT = 0.56;
  function F(hip, hr, foot, ek) { return { hip: hip, t: 180, hd: 0, hr: hr, foot: foot, fd: 90, ek: ek || [1, 0], kk: [1, 0] }; }
  var FK = [
    { at: 0.00, k: F([0, 66], [8, -48], [8, R]) },
    { at: 0.14, k: F([0, 46], [10, -38], [14, R]) },
    { at: 0.30, k: F([0, 112], [16, 42], [22, 46]) },
    { at: 0.46, k: F([0, 106], [18, 40], [20, 40]) },
    { at: 0.60, k: F([0, 50], [16, 28], [22, R]) },
    { at: 0.74, k: F([0, 66], [26, 42], [14, R]) },
    { at: 0.88, k: F([0, 66], [14, 24], [14, R], [1, -1]) },
    { at: 1.00, k: F([0, 66], [26, 44], [14, R]) }
  ];
  var TEMPO = ['chestpress', 'flies', 'dips', 'triext', 'goblet', 'lunge', 'rdl', 'bridge', 'calf', 'situp', 'legraise', 'row1', 'row2', 'superman', 'curl', 'hammer', 'shpress', 'latraise', 'frontraise', 'airsquat', 'pullup', 'uppercut'];
  var EX = [
    { id: 'arms', name: 'Giros de brazos', grp: 'Calentamiento', view: 'side', T: 1800, d: [60, 0, 1], tp: 0.25,
      fn: function (p) { var f = 2 * Math.PI * p; return P(STAND, { hand: [46 * Math.sin(f), 108 - 46 * Math.cos(f)], ek: [-1, 0] }); } },
    { id: 'jacks', name: 'Jumping jacks', grp: 'Calentamiento', view: 'front', T: 1000, d: [40, 15, 3],
      A: P(STAND, { hr: [8, -48], foot: [8, R], ek: [1, 0], kk: [1, 0] }),
      B: P(STAND, { hip: [0, 70], hr: [14, 44], foot: [27, 10], ek: [1, 0], kk: [1, 0] }) },

    { id: 'march', name: 'Marcha elevando rodillas', grp: 'Calentamiento', view: 'side', T: 1200, d: [45, 10, 1],
      A: P(STAND, { hr: [14, -12], ek: [-1, -0.4], foot: [0, R], foot2: [22, 50], kk2: [1, 0.2] }),
      B: P(STAND, { hr: [-14, -12], ek: [-1, 0.2], foot: [22, 50], foot2: [0, R], kk: [1, 0.2] }) },

    { id: 'hamfold', name: 'Flexión adelante (isquiotibiales)', grp: 'Estiramiento', view: 'side', T: 5000, d: [30, 5, 1],
      A: P(STAND, { hr: [4, -48] }),
      B: P(STAND, { hip: [-14, 62], t: 140, hd: 20, hand: [12, 36], ek: [-1, 0], kk: [1, 0.3] }) },
    { id: 'hipflex', name: 'Flexor de cadera (zancada)', grp: 'Estiramiento', view: 'side', T: 5000, d: [25, 5, 2],
      A: P(STAND, { hip: [0, 52], hr: [8, -36], foot: [30, R], foot2: [-44, R], fd2: 270, kk: [1, 0.4], kk2: [0.2, -1] }),
      B: P(STAND, { hip: [8, 44], hr: [8, -36], foot: [30, R], foot2: [-44, R], fd2: 270, kk: [1, 0.4], kk2: [0.2, -1] }) },
    { id: 'tristr', name: 'Estiramiento de tríceps', grp: 'Estiramiento', view: 'side', T: 5000, d: [20, 5, 2],
      A: P(STAND, { hand: [-4, 114], ek: [0, 1], hand2: [-3, 132] }),
      B: P(STAND, { hand: [-6, 108], ek: [0, 1], hand2: [-6, 134] }) },
    { id: 'calfstr', name: 'Estiramiento de pantorrilla', grp: 'Estiramiento', view: 'side', T: 5000, d: [20, 5, 2],
      A: P(STAND, { hip: [-4, 62], t: 125, hand: [44, 92], ek: [0, -1], foot: [18, R], foot2: [-34, R], kk: [1, 0.3], kk2: [0, 1] }),
      B: P(STAND, { hip: [0, 60], t: 120, hand: [44, 92], ek: [0, -1], foot: [18, R], foot2: [-34, R], kk: [1, 0.3], kk2: [0, 1] }),
      bg: function () { return '<g stroke="currentColor" stroke-opacity=".42" stroke-width="4" stroke-linecap="round"><line x1="52" y1="-4" x2="52" y2="-130"/></g>'; }, ext: [-60, 60, 0, 130] },
    { id: 'breath', name: 'Respiración profunda', grp: 'Estiramiento', view: 'side', T: 6000, d: [30, 0, 1],
      A: P(STAND, { hr: [8, -48] }),
      B: P(STAND, { hr: [-6, 46], ek: [-1, 0], hd: -4 }) },

    { id: 'pushup', name: 'Flexiones', grp: 'Pecho y tríceps', view: 'side', T: 2000, d: [40, 45, 3],
      fn: function (p) { return mix(P(PLK, pu0, { hand: [pu0.sh[0], R] }), P(PLK, pu1, { hand: [pu0.sh[0], R] }), pushS(p)); } },
    { id: 'chestpress', name: 'Press de pecho en el piso', grp: 'Pecho y tríceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o',
      A: P(SUP, { hand: [-17, 28], ek: [0, -1] }), B: P(SUP, { hand: [-42, 51], ek: [1, 0] }) },
    { id: 'flies', name: 'Aperturas en el piso', grp: 'Pecho y tríceps', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(SUP, { hand: [-42, 50], ek: [1, -0.2] }), B: P(SUP, { hand: [-14, 24], ek: [0.3, -1] }) },
    { id: 'dips', name: 'Fondos de tríceps en silla', grp: 'Pecho y tríceps', view: 'side', T: 2200, d: [40, 45, 3],
      A: P(STAND, { hip: [-12, 40], sh: [-24, 80], hand: [-26, 34], foot: [38, R], ek: [-1, 0], kk: [1, 1] }),
      B: P(STAND, { hip: [-10, 16], sh: [-22, 56], hand: [-26, 34], foot: [38, R], ek: [-1, 0], kk: [1, 1] }),
      bg: function () { return chair(-48, -14, 28); }, ext: [-50, 40, 0, 32] },
    { id: 'triext', name: 'Extensión de tríceps', grp: 'Pecho y tríceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'v',
      A: P(STAND, { hr: [-10, 6], ek: [1, 0] }), B: P(STAND, { hr: [3, 46], ek: [1, 0] }) },

    { id: 'goblet', name: 'Sentadilla goblet', grp: 'Piernas', view: 'side', T: 2600, d: [40, 45, 3], db: 'v',
      A: P(STAND, { hr: [14, -10], ek: [0, -1] }),
      B: P(STAND, { hip: [-28, 38], t: 150, hr: [14, -10], ek: [0, -1], kk: [1, 0.3] }) },
    { id: 'lunge', name: 'Zancadas con mancuernas', grp: 'Piernas', view: 'side', T: 2800, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [2, -46], foot2: [0, R], fd2: 90, kk2: [1, 0.2] }),
      B: P(STAND, { hip: [0, 40], hr: [2, -46], foot: [32, R], foot2: [-34, 12], fd2: 40, kk: [1, 0.5], kk2: [0.5, -1] }) },
    { id: 'rdl', name: 'Peso muerto rumano', grp: 'Piernas', view: 'side', T: 3000, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -47] }), B: P(HINGE, { hr: [1, -47] }) },
    { id: 'bridge', name: 'Puente de glúteos', grp: 'Piernas', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(SUP, {}), B: P(SUP, { hip: [-12.6, 36], sh: [-42, 6] }) },
    { id: 'calf', name: 'Elevación de talones', grp: 'Piernas', view: 'side', T: 1600, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -48] }), B: P(STAND, { hip: [0, 76], hr: [4, -48], foot: [0, 14], fd: 40 }) },
    { id: 'sidekick', name: 'Patadas laterales', grp: 'Piernas', view: 'front', T: 2400, d: [40, 45, 3],
      A: P(STAND, { hr: [2, -38], ek: [1, 0], foot: [8, R], foot2: [-8, R], fd: 90, fd2: -90 }),
      B: P(STAND, { hr: [2, -38], ek: [1, 0], foot: [46, 13], foot2: [-8, R], fd: 60, fd2: -90 }) },

    { id: 'situp', name: 'Abdominales en silla', grp: 'Core', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(STAND, { hip: [0, 30], t: 295, hd: -10, hr: [10, -6], foot: [44, R], ek: [0, -1], kk: [0, 1] }),
      B: P(STAND, { hip: [0, 30], t: 205, hd: -10, hr: [10, -6], foot: [44, R], ek: [0, -1], kk: [0, 1] }),
      bg: function () {
        return '<g stroke="currentColor" stroke-opacity=".42" stroke-width="4" stroke-linecap="round" fill="none"><line x1="-8" y1="-25" x2="16" y2="-25"/><line x1="-8" y1="-25" x2="-56" y2="-8"/><line x1="4" y1="-25" x2="4" y2="-1"/><line x1="-40" y1="-15" x2="-40" y2="-1"/></g>';
      }, ext: [-62, 50, 0, 40] },
    { id: 'wheel', name: 'Rueda abdominal', grp: 'Core', view: 'side', T: 3000, d: [40, 45, 3],
      A: P(STAND, { hip: [-30, 38], t: 125, hd: -20, hand: [8, 14], foot: [-62, R], fd: 270, ek: [1, -0.4], kk: [0.5, -1] }),
      B: P(STAND, { hip: [-12.3, 33], t: 100, hd: -20, hand: [66, 12], foot: [-62, R], fd: 270, ek: [1, -0.4], kk: [0.5, -1] }),
      fg: function (J) {
        var w = J.armN[2], c = [w[0], 9.5];
        return '<g><circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="9" fill="none" stroke="currentColor" stroke-width="3.4" ' + OUT.replace('stroke="var(--bgc,#fff)" stroke-width="1.6"', '') + '/><circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="2.6" fill="currentColor"/></g>';
      }, ext: [-70, 84, 0, 30] },
    { id: 'legraise', name: 'Elevación de piernas', grp: 'Core', view: 'side', T: 2800, d: [40, 45, 3],
      A: P(SUP, { foot: [64, 9], fd: 160 }), B: P(SUP, { foot: [4, 68], fd: 255 }) },
    { id: 'bicycle', name: 'Bicicleta abdominal', grp: 'Core', view: 'side', T: 1800, d: [40, 45, 3],
      A: P(SUP, { hip: [0, R + 2], sh: [-38, 22], hd: -20, hr: [-14, 6], ek: [0, 1], foot: [22, 34], foot2: [64, 16], kk: [-1, 0.6], kk2: [-1, 0.6] }),
      B: P(SUP, { hip: [0, R + 2], sh: [-38, 22], hd: -20, hr: [-14, 6], ek: [0, 1], foot: [64, 16], foot2: [22, 34], kk: [-1, 0.6], kk2: [-1, 0.6] }) },
    { id: 'sideplank', name: 'Plancha lateral', grp: 'Core', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(PLK, sp0, { hand: [52, R], hand2: [31, 76], ek: [-1, -1], ek2: [-1, 0], fd: 40 }),
      B: P(PLK, sp1, { hip: [sp1.hip[0], sp1.hip[1] - 5], hand: [52, R], hand2: [31, 76], ek: [-1, -1], ek2: [-1, 0], fd: 40 }) },
    { id: 'plank', name: 'Plancha frontal', grp: 'Core', view: 'side', T: 3000, d: [40, 45, 3],
      A: P(PLK, fp0, { hand: [49, R] }), B: P(PLK, fp1, { hand: [49, R] }) },

    { id: 'row1', name: 'Remo a una mano', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o', dbN: true,
      A: P(HINGE, { hand: [15, 26], hand2: [44, 34], ek2: [0, 1] }),
      B: P(HINGE, { hand: [-14, 50], hand2: [44, 34], ek2: [0, 1] }),
      bg: function () { return chair(34, 66, 28); }, ext: [-40, 70, 0, 32] },
    { id: 'row2', name: 'Remo inclinado', grp: 'Espalda y bíceps', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(HINGE, { hand: [15, 25] }), B: P(HINGE, { hand: [-2, 54] }) },
    { id: 'superman', name: 'Superman', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], face: -1,
      A: P(SUP, { hd: 0, hand: [-91, R], foot: [66, R], ek: [0, 1], kk: [0, 1] }),
      B: P(SUP, { hip: [0, R], sh: [-41.4, 11.3], hd: -5, hand: [-87, 21], foot: [64, 16], ek: [0, 1], kk: [0, 1] }) },
    { id: 'curl', name: 'Curl de bíceps', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -48], ek: [-1, -0.2] }), B: P(STAND, { hr: [16, -12], ek: [-1, -0.5] }) },
    { id: 'hammer', name: 'Curl martillo', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'h',
      A: P(STAND, { hr: [4, -48], ek: [-1, -0.2] }), B: P(STAND, { hr: [16, -12], ek: [-1, -0.5] }) },

    { id: 'shpress', name: 'Press de hombros', grp: 'Hombros', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hip: [0, 32], hr: [6, 6], ek: [0, -1], foot: [34, R], kk: [0.5, 1] }),
      B: P(STAND, { hip: [0, 32], hr: [3, 46], ek: [-1, 0], foot: [34, R], kk: [0.5, 1] }),
      bg: function () { return chair(-20, 12, 26, [-20, 72]); }, ext: [-24, 44, 0, 28] },
    { id: 'latraise', name: 'Elevaciones laterales', grp: 'Hombros', view: 'front', T: 2200, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [7, -47], ek: [1, -0.5], foot: [8, R] }), B: P(STAND, { hr: [46, -3], ek: [1, -0.5], foot: [8, R] }) },
    { id: 'frontraise', name: 'Elevaciones frontales', grp: 'Hombros', view: 'side', T: 2200, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [3, -48] }), B: P(STAND, { hr: [47, -2] }) },

    { id: 'rope', name: 'Saltar la cuerda', grp: 'Cardio', view: 'front', T: 700, d: [60, 15, 3],
      A: P(STAND, { hr: [14, -26], ek: [1, -0.5], foot: [8, R] }), B: P(STAND, { hip: [0, 74], hr: [14, -26], ek: [1, -0.5], foot: [8, 12] }),
      fg: function (J, p) {
        var hl = J.armF[2], hr = J.armN[2], ch = 55 + 135 * Math.cos(2 * Math.PI * p);
        return '<path d="M' + pt(hl) + ' Q0,' + n1(-ch) + ' ' + pt(hr) + '" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" opacity=".8"/>';
      }, ext: [-46, 46, -2, 142] },
    { id: 'squatjump', name: 'Sentadillas con salto', grp: 'Cardio', view: 'side', T: 1400, d: [30, 30, 3],
      A: P(STAND, { hip: [-26, 40], t: 145, hr: [-28, -22], ek: [-1, 0], kk: [1, 0.3] }),
      B: P(STAND, { hip: [0, 92], t: 180, hr: [3, 47], foot: [0, 26], fd: 140, ek: [-1, 0], kk: [1, 0.2] }) },
    { id: 'climbers', name: 'Escaladores', grp: 'Cardio', view: 'side', T: 1000, d: [30, 30, 3],
      A: P(CL, { foot: [12, 22], foot2: [-64, 15] }), B: P(CL, { foot: [-64, 15], foot2: [12, 22] }) },

    { id: 'jabcross', name: 'Jab y cruzado', grp: 'Boxeo', view: 'side', T: 1400, d: [45, 15, 3], db: 'g',
      fn: function (p) {
        var a = p < 0.5 ? punch(p * 2) : 0, b = p >= 0.5 ? punch((p - 0.5) * 2) : 0, ab = Math.max(0, a), bb = Math.max(0, b);
        var bob = Math.sin(2 * Math.PI * p * 2) * 0.8;
        return P(BX, { hip: [3.5 * bb - 1.5 * a, 62 + bob], t: 175 - 7 * bb + 3 * ab - 2 * Math.min(0, a) * 4, hd: -2 * (ab + bb),
          hr: [16 + 31 * a, 2 + 4 * ab], hr2: [16 + 31 * b, 2 + 4 * bb], foot2: [-16 - 2 * bb, R + 5 * bb] });
      }, ext: [-30, 80, 0, 130] },
    { id: 'hook', name: 'Gancho', grp: 'Boxeo', view: 'side', T: 1800, d: [45, 15, 3], db: 'g',
      fn: function (p) {
        function hk(q) { /* q 0..1 dentro del golpe: guardia -> carga (codo atrás) -> barrido al frente -> guardia */
          var G = { h: [16, 2], e: [0, -1], t: 175, x: 0 }, W = { h: [2, 16], e: [-1, 0.6], t: 183, x: -3 }, S = { h: [38, 14], e: [0.4, 1], t: 164, x: 6 };
          var k = q < 0.3 ? [G, W, q / 0.3] : q < 0.5 ? [W, S, (q - 0.3) / 0.2] : [S, G, (q - 0.5) / 0.5];
          var f = q >= 0.3 && q < 0.5 ? eBack(k[2], 1.3) : 0.5 - 0.5 * Math.cos(Math.PI * k[2]), A = k[0], B = k[1];
          function m(u, v) { return u + (v - u) * f; }
          return { h: [m(A.h[0], B.h[0]), m(A.h[1], B.h[1])], e: [m(A.e[0], B.e[0]), m(A.e[1], B.e[1])], t: m(A.t, B.t), x: m(A.x, B.x) };
        }
        var l = p < 0.5 ? hk(p * 2) : hk(0), r = p >= 0.5 ? hk((p - 0.5) * 2) : hk(0);
        var tw = p < 0.5 ? l : r;
        var sw = Math.max(Math.abs(l.x), Math.abs(r.x)) / 6;
        return P(BX, { t: tw.t, hip: [tw.x, 62 - 1.2 * sw], hr: l.h, ek: l.e, hr2: r.h, ek2: r.e, foot2: [-16 - 2 * sw, R + 4 * sw] });
      }, ext: [-30, 80, 0, 130] },
    { id: 'uppercut', name: 'Uppercut', grp: 'Boxeo', view: 'side', T: 1400, d: [45, 15, 3], db: 'g',
      A: P(BX, { hip: [0, 56], hr: [12, -24], hr2: [16, 2], ek: [0, -1] }),
      B: P(BX, { hip: [0, 64], hr: [24, 26], hr2: [16, 2], ek: [0, -1] }), ext: [-30, 70, 0, 150] },
    { id: 'slip', name: 'Esquiva', grp: 'Boxeo', view: 'front', T: 2200, d: [30, 15, 3], db: 'g',
      fn: function (p) {
        var q = (p * 2) % 1, dir = p < 0.5 ? 1 : -1, a;   // se agacha y esquiva al lado, rebota, vuelve
        if (q < 0.1) a = -0.12 * eIO(q / 0.1); else if (q < 0.4) a = -0.12 + 1.12 * eBack((q - 0.1) / 0.3, 1.2); else if (q < 0.55) a = 1; else a = 1 - eIO((q - 0.55) / 0.4);
        var dip = Math.max(0, a);
        return P(BXF, { hip: [-5 * a * dir, 58 - 5 * dip], t: 180 + 15 * a * dir, hd: -6 * a * dir });
      } },
    { id: 'boxstep', name: 'Juego de pies', grp: 'Boxeo', view: 'side', T: 800, d: [30, 30, 3], db: 'g',
      A: P(BX, { hip: [0, 63], foot: [16, R], foot2: [-14, 9], fd2: 40 }),
      B: P(BX, { hip: [0, 60], foot: [10, 9], fd: 40, foot2: [-20, R] }) },

    { id: 'airsquat', name: 'Sentadilla sin peso', grp: 'Calistenia', view: 'side', T: 2400, d: [40, 20, 3],
      A: P(STAND, { hr: [44, -6], ek: [0, 1] }),
      B: P(STAND, { hip: [-28, 38], t: 150, hr: [44, -6], ek: [0, 1], kk: [1, 0.3] }) },
    { id: 'burpee', name: 'Burpee', grp: 'Calistenia', view: 'side', T: 3000, d: [30, 30, 3],
      fn: function (p) { return keysT(BURPEE_T, p); }, ext: [-70, 70, 0, 190] },
    { id: 'deadbug', name: 'Bicho muerto', grp: 'Calistenia', view: 'side', T: 3000, d: [40, 20, 3], face: -1,
      A: P(SUP, { hip: [0, R + 2], hand: [-42, 52], hand2: [-90, 8], ek: [0, 1], ek2: [0, 1], foot: [30, 36], foot2: [64, 16], kk: [-1, 0.6], kk2: [-1, 0.6] }),
      B: P(SUP, { hip: [0, R + 2], hand: [-90, 8], hand2: [-42, 52], ek: [0, 1], ek2: [0, 1], foot: [64, 16], foot2: [30, 36], kk: [-1, 0.6], kk2: [-1, 0.6] }) },
    { id: 'pullup', name: 'Dominadas', grp: 'Calistenia', view: 'side', T: 3200, d: [30, 45, 3],
      A: P(STAND, { hip: [0, 79], sh: [0, 121], hd: 0, hand: [0, 170], foot: [3, 14.5], fd: 120, ek: [-1, 0], kk: [1, 0.2] }),
      B: P(STAND, { hip: [0, 108], sh: [0, 150], hd: -8, hand: [0, 170], foot: [-12, 46], fd: 120, ek: [-1, -0.3], kk: [1, -0.2] }),
      bg: function () { return '<g stroke="currentColor" stroke-opacity=".5" stroke-linecap="round"><line x1="-40" y1="-170" x2="40" y2="-170" stroke-width="5"/><line x1="-40" y1="-170" x2="-40" y2="0" stroke-width="3.5" stroke-opacity=".3"/><line x1="40" y1="-170" x2="40" y2="0" stroke-width="3.5" stroke-opacity=".3"/></g>'; }, ext: [-50, 50, 0, 180] },
    { id: 'wallsit', name: 'Sentadilla en pared', grp: 'Calistenia', view: 'side', T: 4000, d: [30, 30, 3],
      A: P(STAND, { hip: [-26, 40], sh: [-26, 82], hd: 0, hand: [4, 48], foot: [8, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] }),
      B: P(STAND, { hip: [-26, 41], sh: [-26, 83], hd: 0, hand: [4, 48], foot: [8, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] }),
      bg: function () { return '<g stroke="currentColor" stroke-opacity=".42" stroke-width="4" stroke-linecap="round"><line x1="-40" y1="0" x2="-40" y2="-130"/></g>'; }, ext: [-46, 40, 0, 130] },

    { id: 'victory', name: 'Victoria', grp: 'Calistenia', hide: true, T: 5200, d: [30, 0, 1],
      view: function (p) { return p < VSPLIT ? 'side' : 'front'; },
      fn: function (p) { return p < VSPLIT ? keysT(VK, p / VSPLIT) : keysT(FK, (p - VSPLIT) / (1 - VSPLIT)); },
      expr: function (p) {
        if (p < VSPLIT) { var a = p / VSPLIT; return a < 0.2 ? 'effort' : a < 0.62 ? 'shout' : 'smile'; }
        var q = (p - VSPLIT) / (1 - VSPLIT); return q < 0.12 ? 'neutral' : q < 0.55 ? 'shout' : 'smile';
      },
      fg: function (J, p) {
        var o = '', cols = ['var(--acc,#ffc61a)', '#ffd60a', 'currentColor', '#ffffff'];
        var bursts = [[0.22 * VSPLIT, VSPLIT * 0.95, 0], [VSPLIT + 0.22 * (1 - VSPLIT), 1.0, 7]];
        bursts.forEach(function (bz) {
          var t = (p - bz[0]) / (bz[1] - bz[0]); if (t <= 0) return; if (t > 1) t = 1;
          for (var i = 0; i < 26; i++) {
            var a = ((i + bz[2]) * 2.399) % (Math.PI), vx = Math.cos(a + 0.2) * (30 + (i * 37) % 55) * (i % 2 ? 1 : -1), vy = 70 + (i * 53) % 70;
            var x = vx * t * 1.9, y = 150 + vy * t * 1.5 - 170 * t * t;
            if (y < 4) y = 4;
            o += '<rect x="' + n1(x - 2) + '" y="' + n1(-y - 1.2) + '" width="4" height="2.4" fill="' + cols[(i + bz[2]) % 4] + '" transform="rotate(' + n1(i * 40 + t * 520) + ' ' + n1(x) + ' ' + n1(-y) + ')"/>';
          }
        });
        return o;
      }, ext: [-110, 110, 0, 230] },

    { id: 'grip', name: 'Apretador de manos', grp: 'Agarre', view: 'side', T: 1400, d: [30, 30, 3],
      A: P(STAND, { hr: [22, -22], ek: [-1, -0.6] }), B: P(STAND, { hr: [22, -22], ek: [-1, -0.6] }),
      fg: function (J, p, s) {
        var w = J.armN[2], g = 8.5 - 5.5 * s, a = [w[0] - 5, w[1] + g], b = [w[0] - 5, w[1] - g], c = [w[0] + 15, w[1]];
        var d = 'M' + pt(a) + ' L' + pt(c) + ' L' + pt(b);
        return '<path d="' + d + '" fill="none" stroke="var(--bgc,#fff)" stroke-width="8" stroke-linecap="round" stroke-linejoin="round"/><path d="' + d + '" fill="none" stroke="currentColor" stroke-width="4.4" stroke-linecap="round" stroke-linejoin="round"/>';
      } }
  ];

  var BYID = {}, GROUPS = [];
  EX.forEach(function (e) { BYID[e.id] = e; if (TEMPO.indexOf(e.id) >= 0 && !e.fn) e.tempo = true; if (GROUPS.indexOf(e.grp) < 0) GROUPS.push(e.grp); });

  // Ritmo natural de repetición: fase lenta, pausa al final, regreso con rebote leve, respiro al inicio
  function tempoS(p) {
    if (p < 0.45) return eIO(p / 0.45);
    if (p < 0.52) return 1;
    if (p < 0.92) return 1 - eBack((p - 0.52) / 0.4, 0.7);
    return 0;
  }
  function viewOf(ex, p) { return typeof ex.view === 'function' ? ex.view(p) : ex.view; }
  function pose(ex, p) {
    if (ex.fn) return ex.fn(p);
    var s = ex.tempo ? tempoS(p) : 0.5 - 0.5 * Math.cos(2 * Math.PI * p);
    return mix(ex.A, ex.B, s);
  }
  function sOf(p) { return 0.5 - 0.5 * Math.cos(2 * Math.PI * p); }

  var vbCache = {};
  function bbox(ex, sex) {
    var key = ex.id + sex; if (vbCache[key]) return vbCache[key];
    var x0 = 1e9, x1 = -1e9, h0 = 0, h1 = 0, i, B = BODY[sex];
    function acc(c, r) { r = r || 6; x0 = Math.min(x0, c[0] - r); x1 = Math.max(x1, c[0] + r); h0 = Math.min(h0, c[1] - r); h1 = Math.max(h1, c[1] + r); }
    for (i = 0; i <= 48; i++) {
      var J = solve(pose(ex, i / 48), viewOf(ex, i / 48), sex);
      [J.hip, J.sh, J.armN[1], J.armN[2], J.armF[1], J.armF[2], J.legN[1], J.legN[2], J.legN[3], J.legF[1], J.legF[2], J.legF[3]].forEach(function (c) { acc(c, 7); });
      acc(J.head, B.head + 12);
      if (J.view === 'front') { acc([J.sh[0] + B.sw, J.sh[1]], 6); acc([J.sh[0] - B.sw, J.sh[1]], 6); }
    }
    if (ex.ext) { x0 = Math.min(x0, ex.ext[0]); x1 = Math.max(x1, ex.ext[1]); h0 = Math.min(h0, ex.ext[2]); h1 = Math.max(h1, ex.ext[3]); }
    h0 = Math.min(h0, -6);
    var pad = 14, w = x1 - x0 + pad * 2, h = h1 - h0 + pad * 2, cx = (x0 + x1) / 2, AR = 1.8;
    if (w / h < AR) w = h * AR; else h = w / AR;
    var bottom = h0 - pad;          // el piso queda abajo
    var vb = { x: cx - w / 2, y: -(bottom + h), w: w, h: h, x0: x0, x1: x1 };
    return (vbCache[key] = vb);
  }

  function viewBox(id, sex) { var v = bbox(BYID[id], sex); return n1(v.x) + ' ' + n1(v.y) + ' ' + n1(v.w) + ' ' + n1(v.h); }
  function render(id, sex, p, still, expr, hl) {
    var ex = BYID[id], v = bbox(ex, sex), s = sOf(p);
    var J = solve(pose(ex, p), viewOf(ex, p), sex);
    var pp = p - 0.03, J0 = solve(pose(ex, pp < 0 ? pp + 1 : pp), viewOf(ex, p), sex);
    J.hv = [(J.head[0] - J0.head[0]) * (pp < 0 ? 0 : 1), (J.head[1] - J0.head[1]) * (pp < 0 ? 0 : 1)];
    var ground = '<line x1="' + n1(v.x - 400) + '" y1="1.5" x2="' + n1(v.x + v.w + 400) + '" y2="1.5" stroke="currentColor" stroke-opacity=".38" stroke-width="3"/>';
    return ground + drawFigure(J, ex, sex, p, s, expr, hl);
  }
  function thumb(id, sex) {
    var ex = BYID[id], p = ex.tp === undefined ? 0.5 : ex.tp;
    return '<svg viewBox="' + viewBox(id, sex) + '" preserveAspectRatio="xMidYMid meet" aria-hidden="true" focusable="false">' + render(id, sex, p) + '</svg>';
  }
  function check() { // diagnóstico: alcance de brazos/piernas
    var rep = [];
    EX.forEach(function (ex) {
      ['m', 'f'].forEach(function (sex) {
        var worst = 0;
        for (var i = 0; i <= 24; i++) worst = Math.max(worst, solve(pose(ex, i / 24), viewOf(ex, i / 24), sex).short);
        if (worst > 0.8) rep.push(ex.id + '/' + sex + ': ' + worst.toFixed(1));
      });
    });
    return rep;
  }

  root.LochFig = { list: EX, byId: BYID, groups: GROUPS, viewBox: viewBox, render: render, thumb: thumb, check: check, muscles: muscles, T: function (id) { return BYID[id].T; } };
})(window);

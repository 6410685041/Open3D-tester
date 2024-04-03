use colors_transform::{Color, Rgb};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, prelude::*, BufReader};
use std::io::{BufWriter, Write};

struct PointCloudInfo {
    pntx: Vec<Point>,
    x0: f32,
    x1: f32,
    y0: f32,
    y1: f32,
    grpm: HashMap<usize, PointCloudGrp>,
}

#[derive(Clone, Default)]
struct Point {
    x: f32,
    y: f32,
    z: f32,
    r: u8,
    g: u8,
    b: u8,
    hue: f32,
}

#[derive(Clone, Default)]
struct PointCloudGrp {
    xi: usize,
    yi: usize,
    memb: Vec<usize>,
}

#[derive(Clone, Default)]
struct AdjPnt {
    xi: usize,
    ds: f32,
}

#[derive(Clone, Default)]
struct AreaEx {
    x0: f32,
    x1: f32,
    y0: f32,
    y1: f32,
    ar: f32,
}

const BOXWD: f32 = 1.0f32; // 1 meter
const ADJMX: f32 = 0.6f32; // within 60 cm
const GRDMX: f32 = 0.1f32; // gradian 0.1
const ADJNO: usize = 1; // min adjacent
const HUEMX: f32 = 5_f32;
const HUEXX: f32 = 365.0 - HUEMX;
const CNTMX: usize = 1000;
const AREAM: f32 = 100.0;

impl AreaEx {
    fn adjust(&mut self, x: f32, y: f32) {
        if x < self.x0 {
            self.x0 = x;
        }
        if x > self.x1 {
            self.x1 = x;
        }
        if y < self.y0 {
            self.y0 = y;
        }
        if y > self.y1 {
            self.y1 = y;
        }
        self.area();
    }
    fn area(&mut self) {
        self.ar = (self.x1 - self.x0) * (self.y1 - self.y0);
    }
}

fn main() {
    if let Ok(mut pci) = read("../data/kota_circuit3.ply") {
        proc2(&mut pci);
    }
}

fn proc2(pci: &mut PointCloudInfo) {
    // divide into groups BEGIN
    let xcn = (pci.x1 - pci.x0) / BOXWD;
    let ycn = (pci.y1 - pci.y0) / BOXWD;
    let xcn = xcn as usize + 1;
    let ycn = ycn as usize + 1;
    for i in 0..pci.pntx.len() {
        let xi = (pci.pntx[i].x - pci.x0) / BOXWD;
        let xi = xi as usize;
        let yi = (pci.pntx[i].y - pci.y0) / BOXWD;
        let yi = yi as usize;
        let gk = yi * xcn + xi;
        if let Some(pcg) = pci.grpm.get_mut(&gk) {
            pcg.memb.push(i);
        } else {
            let memb = vec![i];
            pci.grpm.insert(gk, PointCloudGrp { xi, yi, memb });
        }
    }
    print!("pci g: {}\n", pci.grpm.len());
    print!("xn:{} yn:{}\n", xcn, ycn);

    let mut gszm = HashMap::new();
    for (_k, pcg) in &pci.grpm {
        let cn = pcg.memb.len();
        if let Some(cn) = gszm.get_mut(&cn) {
            *cn += 1;
        } else {
            gszm.insert(cn, 1);
        }
    }
    print!("NEW: {}\n", pci.grpm.len());
    // divide into groups END

    // check continuous of points
    let mut conns = vec![Vec::<AdjPnt>::new(); pci.pntx.len()];
    for (_k, pcg) in &pci.grpm {
        let mut xks = vec![];
        // for xi in pcg.xi - 1..=(pcg.xi + 1) {
        //     for yi in pcg.yi - 1..(pcg.yi + 1) {
        for xi in pcg.xi.checked_sub(1).unwrap_or(0)..=(pcg.xi + 1) {
            for yi in pcg.yi.checked_sub(1).unwrap_or(0)..=(pcg.yi + 1) {
                let kx = yi * xcn + xi;
                if let Some(_) = pci.grpm.get(&kx) {
                    xks.push(kx);
                }
            }
        }
        for mi in &pcg.memb {
            let pnt = &pci.pntx[*mi];
            for kx in &xks {
                if let Some(pcg) = pci.grpm.get(&kx) {
                    for xi in &pcg.memb {
                        if mi == xi {
                            continue;
                        }
                        let pnx = &pci.pntx[*xi];
                        let dx = (pnt.x - pnx.x).abs();
                        let dy = (pnt.y - pnx.y).abs();
                        let dz = (pnt.z - pnx.z).abs();

                        // check adjacent
                        if dx < ADJMX && dy < ADJMX {
                            let xi = *xi;
                            let ds = dx.max(dy);
                            let gd = dz / ds;
                            // check vertical gradiant
                            if gd < GRDMX {
                                let dhu = (pnt.hue - pnx.hue).abs();
                                // check colour Hue
                                if dhu < HUEMX {
                                    conns[*mi].push(AdjPnt { xi, ds });
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    let mut cnm = HashMap::new();
    let mut set1 = vec![false; conns.len()];
    for i in 0..pci.pntx.len() {
        if conns[i].len() > 0 {
            let adc = conns[i].len();
            conns[i].sort_by(|a, b| a.ds.partial_cmp(&b.ds).unwrap());
            if let Some(cn) = cnm.get_mut(&adc) {
                *cn += 1;
            } else {
                cnm.insert(adc, 1);
            }
            // number of points
            if adc >= ADJNO {
                set1[i] = true;
            }
        }
    }
    print!("ADJ\n");
    for i in 1..10 {
        if let Some(cn) = cnm.get(&i) {
            print!(" {} - {}\n", i, cn);
        }
    }

    let mut mark = vec![-1i32; pci.pntx.len()];
    let mut cnts = vec![0usize; pci.pntx.len()];
    let mut areas = vec![AreaEx::default(); pci.pntx.len()];
    for i in 0..mark.len() {
        let ii: i32 = i as i32;
        if set1[i] == false {
            continue;
        }
        if mark[i] >= 0 {
            continue;
        }
        if conns[i].len() == 0 {
            continue;
        }
        mark[i] = ii;
        cnts[i] += 1;
        let (x0, y0) = (pci.pntx[i].x, pci.pntx[i].y);
        let (x1, y1) = (x0, y0);
        areas[i] = AreaEx {
            x0,
            x1,
            y0,
            y1,
            ..Default::default()
        };
        let mut bran: Vec<usize> = vec![];
        for adj in &conns[i] {
            bran.push(adj.xi);
        }
        while let Some(ij) = bran.pop() {
            if set1[ij] == false {
                continue;
            }
            if mark[ij] >= 0 {
                continue;
            }
            mark[ij] = ii;
            cnts[i] += 1;
            areas[i].adjust(pci.pntx[ij].x, pci.pntx[ij].y);
            for adj in &conns[ij] {
                bran.push(adj.xi);
            }
        }
    }

    let mut cn = 0;
    let mut pnts = Vec::new();
    let mut area = Vec::new();
    for i in 0..mark.len() {
        if cnts[i] >= CNTMX && areas[i].ar > AREAM {
            cn += 1;
            area.push(i);
            //print!("A: {} {}\n", i, areas[i].ar);
        }
        if mark[i] >= 0 {
            let gi = mark[i] as usize;
            // check area size
            if cnts[gi] >= CNTMX && areas[gi].ar > AREAM {
                pnts.push(pci.pntx[i].clone());
            }
        }
    }
    write("../data/kota-3.ply", &pnts);
    print!("FIN {}\n", cn);
}

// write to ASCII PLY
fn write(f: &str, pntx: &Vec<Point>) {
    let hd = format!(
        r###"ply
format ascii 1.0
comment Created by TUENG Special Topic 2024-03-28
element vertex {}
property double x
property double y
property double z
property uchar red
property uchar green
property uchar blue
end_header
"###,
        pntx.len()
    );
    if let Ok(file) = File::create(f) {
        let mut file = BufWriter::new(file);
        file.write_all(hd.as_bytes()).unwrap();
        for p in pntx {
            let l = format!("{} {} {} {} {} {}\n", p.x, p.y, p.z, p.r, p.g, p.b);
            file.write_all(l.as_bytes()).unwrap();
        }
    }
}

fn read(f: &str) -> io::Result<PointCloudInfo> {
    let file = File::open(f)?;
    let reader = BufReader::new(file);
    let mut pnts = Vec::new();
    let mut pntx = Vec::new();

    let (mut x0, mut x1, mut y0, mut y1, mut z0, mut z1) =
        (0_f32, 0_f32, 0_f32, 0_f32, 0_f32, 0_f32);
    for line in reader.lines() {
        if let Ok(line) = line {
            let line = line.as_str();
            let parts = line.split(" ");
            let wds = parts.collect::<Vec<&str>>();
            if wds.len() == 6 {
                if let (Ok(x), Ok(y), Ok(z), Ok(r), Ok(g), Ok(b)) = (
                    wds[0].parse::<f32>(),
                    wds[1].parse::<f32>(),
                    wds[2].parse::<f32>(),
                    wds[3].parse::<u8>(),
                    wds[4].parse::<u8>(),
                    wds[5].parse::<u8>(),
                ) {
                    if pnts.len() == 0 {
                        x0 = x;
                        x1 = x;
                        y0 = y;
                        y1 = y;
                        z0 = z;
                        z1 = z;
                    } else {
                        x0 = if x < x0 { x } else { x0 };
                        y0 = if y < y0 { y } else { y0 };
                        z0 = if z < z0 { z } else { z0 };
                        x1 = if x > x1 { x } else { x1 };
                        y1 = if y > y1 { y } else { y1 };
                        z1 = if z > z1 { z } else { z1 };
                    }
                    let rgb = Rgb::from(r as f32, g as f32, b as f32);
                    let hue = rgb.get_hue();
                    let hue = if hue > HUEXX { hue - 365.0 } else { hue };
                    pnts.push((x, y, z, r, g, b, hue));
                    pntx.push(Point {
                        x,
                        y,
                        z,
                        r,
                        g,
                        b,
                        hue,
                    });
                    //print!("{},{},{}\n", x, y, z);
                } else {
                    print!("NG.3\n");
                }
            } else if wds.len() == 3 && wds[0] == "element" && wds[1] == "vertex" {
                let cnt = wds[2].parse::<usize>().unwrap();
                pnts = Vec::with_capacity(cnt);
                pntx = Vec::with_capacity(cnt);
                //print!("cap: {}\n", cnt);
            }
        }
    }
    let grpm = HashMap::new();
    Ok(PointCloudInfo {
        pntx,
        x0,
        x1,
        y0,
        y1,
        grpm,
    })
}

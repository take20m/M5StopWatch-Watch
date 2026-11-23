import { createOpenSCAD } from "openscad-wasm-prebuilt";
import fs from "fs";
const [,, inp, out, ...defs] = process.argv;
const o = await createOpenSCAD({ print: ()=>{}, printErr: (t)=>{ if(/ERROR|WARNING: [^I]/.test(t)) console.error(t);} });
const m = o.getInstance();
m.FS.writeFile("/in.scad", fs.readFileSync(inp, "utf8"));
const args=["/in.scad","-o","/out.stl"];for(const d of defs){args.push("-D",d);}
const rc = m.callMain(args); console.log(out,"rc",rc);
fs.writeFileSync(out, m.FS.readFile("/out.stl"));

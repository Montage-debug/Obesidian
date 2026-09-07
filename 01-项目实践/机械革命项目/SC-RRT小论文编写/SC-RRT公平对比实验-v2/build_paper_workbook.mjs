import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Workbook, SpreadsheetFile } from '@oai/artifact-tool';

const root=path.dirname(fileURLToPath(import.meta.url));
const out=path.join(root,'outputs/paper_completion_20260907');
const input=JSON.parse(await fs.readFile(path.join(out,'sources/workbook_data.json'),'utf8'));
if (!input.matched_ssfor) throw new Error('匹配补跑未完成，拒绝导出最终工作簿');
const wb=Workbook.create();
const previews=path.join(root,'results/paper_package_qa/workbook_previews');
await fs.mkdir(previews,{recursive:true});
const algs=['RRT','RRT-Connect','RRT*','Informed-RRT*','Dynamic-RRT','SC-RRT'];
const names={'Dynamic-RRT':'Dynamic-RRT（本地改写）','SC-RRT-no-ADCS':'无知情采样/反馈整组',
 'SC-RRT-no-SSFOR':'固定参数（旧：提前启动）','SC-RRT-fixed-matched':'固定参数（匹配启动）'};
const metricNames={first_solution_cost_mm:['首解路径长度','mm','0.00'],
 first_solution_time_s:['首解时间','s','0.0000'],first_solution_attempt:['首解扩展尝试','次','0.0'],
 first_solution_nodes:['首解树节点','个','0.0'],first_solution_efficiency:['首解路径效率','比值','0.0000'],
 fixed_budget_cost_mm:['终值路径长度','mm','0.00'],fixed_budget_time_s:['至5000次总时间','s','0.0000'],
 total_nodes:['终值树节点','个','0.0'],collision_checks:['已计数边检查','次','0.0'],
 fixed_budget_efficiency:['终值路径效率','比值','0.0000'],success:['成功比例','比值','0.00%'],
 efficiency_all_runs:['终值效率（未解=0）','比值','0.0000'],first_attempt_capped:['受限首解努力量','次','0.0']};
const sheets=[];
function col(n){let r='';for(;n>0;n=Math.floor((n-1)/26))r=String.fromCharCode(65+(n-1)%26)+r;return r;}
function tableSheet(name,headers,rows,widths,note){
 const sh=wb.worksheets.add(name);sh.showGridLines=false;
 const end=col(headers.length),last=rows.length+1;
 const area=sh.getRange(`A1:${end}${last}`);
 area.values=[headers,...rows];
 area.format.font={name:'Noto Sans CJK SC',size:10,color:'#202B33'};
 area.format.rowHeight=26;area.format.verticalAlignment='center';
 if(headers.length===2){area.format.wrapText=true;area.format.rowHeight=44;}
 const h=sh.getRange(`A1:${end}1`);
 h.format.fill='#344756';h.format.font={name:'Noto Sans CJK SC',size:10,bold:true,color:'#FFFFFF'};
 h.format.wrapText=true;h.format.rowHeight=38;
 widths.forEach((w,i)=>{sh.getRange(`${col(i+1)}1:${col(i+1)}${last}`).format.columnWidth=w;});
 if(rows.length>20)sh.freezePanes.freezeRows(1);
 if(note){sh.getRange(`A${last+2}`).values=[[note]];sh.getRange(`A${last+2}:${end}${last+2}`).format.font={name:'Noto Sans CJK SC',size:10,color:'#53616D'};}
 sheets.push({name,range:`A1:${end}${headers.length===2?last:Math.min(last,14)}`});
 return sh;
}

tableSheet('使用说明',['项目','说明'],[
 ['实验范围','静态点机器人；二维圆障碍与三维球障碍，不等同六自由度机械臂。'],
 ['主实验','6个实现 × 2个维度 × 10张地图 × 50个种子 = 6000次。'],
 ['历史消融','3个配置 × 2个维度 × 10张地图 × 50个种子 = 3000次。'],
 ['匹配SSFOR','2个配置 × 2个维度 × 10张地图 × 50个种子 = 2000次。'],
 ['样本关系','重复运行的完整SC-RRT不能合并成独立新样本。地图是聚类单位。'],
 ['数值口径','均值、样本SD、中位数、Q1、Q3分别存为数值；有效n注明缺失。'],
 ['失败处理','路径指标与首解时间条件于成功；总时间/节点包含失败；未解效率另列0。'],
 ['统计方法','地图→配对种子分层bootstrap 10000次；地图均值差精确signed-rank。'],
 ['多重检验','Holm：每批次、每指标内覆盖参照算法及两个维度。'],
 ['方向','统计检验页仅路径长度：正改善代表SC-RRT更短。'],
 ['Dynamic标签','第六实现是本地改写，不是原作者严格复现；请保留此标签。'],
 ['ADCS范围','整组关闭知情采样及反馈，未单独隔离非对称几何因素。'],
 ['SSFOR选择','正文优先使用匹配启动；历史提前启动的固定参数行仅供追溯。'],
 ['时间边界','5000次扩展不等于同秒数；匹配批次时间仅作描述性参考。'],
 ['原始来源','sources/reproduction/results下三批原始记录；冻结地图、配置和元数据随包保存。'],
 ['数值刷新','复杂统计由可复现脚本生成；本工作簿是静态结果。简单差值为公式。'],
 ['图与写作','同目录figures、论文写作指南.md、结果解读.md。'],
 ['显著性与贡献','统计显著不等于工程提升很大，不代表SCI录用保证。'],
],[27,105]);

const headers=['维度','算法/配置','指标','单位','总N','成功数','有效n','均值','样本SD','中位数','Q1','Q3'];
function resultSheet(name,data,filter,order=algs){
 const selected=data.filter(filter).filter(x=>metricNames[x.metric]).sort((a,b)=>a.dimension-b.dimension||order.indexOf(a.algorithm)-order.indexOf(b.algorithm)||Object.keys(metricNames).indexOf(a.metric)-Object.keys(metricNames).indexOf(b.metric));
 const rows=selected.map(r=>[`${r.dimension}D`,names[r.algorithm]||r.algorithm,metricNames[r.metric][0],metricNames[r.metric][1],r.n,r.successful_n,r.valid_n,r.mean,r.std,r.median,r.q1,r.q3]);
 const sh=tableSheet(name,headers,rows,[8,35,30,9,10,10,10,15,15,15,15,15]);
 sh.getRange(`E2:G${rows.length+1}`).setNumberFormat('0');
 selected.forEach((r,i)=>sh.getRange(`H${i+2}:L${i+2}`).setNumberFormat(metricNames[r.metric][2]));
 return sh;
}
resultSheet('二维主对比',input.main,r=>r.dimension===2);
resultSheet('三维主对比',input.main,r=>r.dimension===3);
const terminal=new Set(['fixed_budget_cost_mm','fixed_budget_time_s','total_nodes','collision_checks','fixed_budget_efficiency','success']);
resultSheet('整组消融',input.legacy_ablation,r=>r.algorithm!=='SC-RRT-no-SSFOR'&&terminal.has(r.metric),['SC-RRT','SC-RRT-no-ADCS']);
resultSheet('匹配SSFOR',input.matched_ssfor,r=>terminal.has(r.metric),['SC-RRT','SC-RRT-fixed-matched']);
resultSheet('历史固定对照',input.legacy_ablation,r=>r.algorithm!=='SC-RRT-no-ADCS'&&terminal.has(r.metric),['SC-RRT','SC-RRT-no-SSFOR']);

const costs=[];
for(const [dataset,items] of Object.entries(input.comparisons))for(const r of items){
 if(r.metric!=='fixed_budget_cost_mm')continue;
 if(dataset==='legacy_ablation'&&r.reference==='SC-RRT-no-SSFOR')continue;
 costs.push([dataset,r.dimension,names[r.reference]||r.reference,r.paired_n,r.map_n,r.reference_mean,r.sc_mean,null,null,r.hierarchy_pct_ci_low/100,r.hierarchy_pct_ci_high/100,r.sc_better_maps,r.map_wilcoxon_p_holm]);
}
const cp=tableSheet('路径长度统计',['批次','维度','参照配置','配对n','地图数','参照均值(mm)','SC均值(mm)','参照−SC(mm)','相对改善','CI下限','CI上限','改善地图数','Holm p'],costs,[23,8,35,10,10,17,17,18,15,15,15,15,14]);
cp.getRange(`F2:H${costs.length+1}`).setNumberFormat('0.00');
cp.getRange(`I2:K${costs.length+1}`).setNumberFormat('0.00%');
cp.getRange(`M2:M${costs.length+1}`).setNumberFormat('0.000000');
cp.getRange(`H2:H${costs.length+1}`).formulas=costs.map((r,i)=>[`=F${i+2}-G${i+2}`]);
cp.getRange(`I2:I${costs.length+1}`).formulas=costs.map((r,i)=>[`=H${i+2}/F${i+2}`]);

const maps=[];
for(const [dataset,items] of Object.entries(input.per_map))for(const r of items){
 if(dataset==='main'||r.metric!=='fixed_budget_cost_mm'||r.reference==='SC-RRT-no-SSFOR')continue;
 maps.push([dataset,r.dimension,names[r.reference]||r.reference,r.environment_id,r.paired_n,r.reference_mean,r.sc_mean,null]);
}
const mp=tableSheet('消融各地图',['批次','维度','参照配置','地图','配对n','参照均值(mm)','SC均值(mm)','相对改善'],maps,[23,8,35,25,10,18,18,16]);
mp.getRange(`F2:G${maps.length+1}`).setNumberFormat('0.00');
mp.getRange(`H2:H${maps.length+1}`).setNumberFormat('0.00%');
mp.getRange(`H2:H${maps.length+1}`).formulas=maps.map((r,i)=>[`=(F${i+2}-G${i+2})/F${i+2}`]);

tableSheet('参数与协议',['项目','取值或定义'],[
 ['空间','2D: [0,1500]² mm；3D: [0,1500]³ mm'],['起终点','各坐标75 mm → 各坐标1425 mm'],
 ['障碍','2D:225圆，半径15–35 mm；3D:400球，半径30–75 mm；允许重叠'],
 ['评价设计','每维度10地图×每图50种子；uniform/central/slab = 4/3/3张'],
 ['扩展预算','5000次；CONNECT每一步都扣预算；非等时间预算'],['步长/目标阈值','30 mm / 45 mm；碰撞膨胀0 mm'],
 ['检查点','100,250,500,1000,2000,3000,4000,5000'],['路径效率','起终点欧氏直线距离 / 实际折线路径长度'],
 ['SC局部重布线','半径75 mm；RRT*共享基线邻域上限180 mm'],['SC连接','触发距离360 mm，最多8步'],
 ['SC采样控制','双侧共享gamma与p；两侧椭球几何由各侧代价构造'],['反馈周期/窗口','至少间隔50扩展；窗口50次反馈观察；第51次观察后方可启用'],
 ['PID','Kp=2, Ki=0.2, Kd=0.8; rho_y=0.9, rho_d=0.8'],['参数范围','gamma∈[1,4], p∈[0.2,0.95], 积分∈[-3,3]'],
 ['匹配固定对照','与完整方法同启动；启用后gamma=1.5, p=0.8'],['跨树偏置','加权候选偏置，不是严格Pareto前沿计算'],
 ['Dynamic改写','interval=8, non-Pareto=0.1, goal bias=0.15, inflation=1.2'],
 ['后处理','主表无平滑/捷径化；只验证几何路径'],['计算平台','Intel Core i5-14500HX；各批Python/NumPy详见原始metadata'],
],[27,105]);

const checks=[];
for(const s of sheets){
 const preview=await wb.render({sheetName:s.name,range:s.range,scale:1,format:'png'});
 await fs.writeFile(path.join(previews,`${s.name}.png`),new Uint8Array(await preview.arrayBuffer()));
 checks.push((await wb.inspect({kind:'table',range:`'${s.name}'!${s.range}`,include:'values,formulas',tableMaxRows:3,tableMaxCols:6,maxChars:1800})).ndjson);
 console.log('Rendered',s.name);
}
const errors=await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:30},summary:'Final formula error scan'});
await fs.writeFile(path.join(previews,'inspection.txt'),checks.join('\n')+'\n'+errors.ndjson);
console.log(errors.ndjson);
for(let i=0;i<costs.length;i++){
 const r=i+2,expected=(costs[i][5]-costs[i][6])/costs[i][5];
 const value=cp.getRange(`I${r}`).values[0][0];
 if(typeof value!=='number'||Math.abs(value-expected)>1e-10)throw new Error(`Formula mismatch row ${r}`);
}
await (await SpreadsheetFile.exportXlsx(wb)).save(path.join(out,'SC-RRT_二维三维对比与消融.xlsx'));
console.log('Saved workbook');

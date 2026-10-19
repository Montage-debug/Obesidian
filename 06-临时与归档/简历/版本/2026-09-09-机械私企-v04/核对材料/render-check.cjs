const {chromium}=require('C:/Users/26434/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const fs=require('fs'),path=require('path'),assert=require('assert');
const {pathToFileURL}=require('url');
(async()=>{
 const out=path.dirname(__dirname),name='郭兴雨-机械设计与机电研发-单页简历';
 const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
 try{
 const context=await browser.newContext({viewport:{width:1000,height:1250},acceptDownloads:true});
 const page=await context.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto(pathToFileURL(path.join(out,name+'.html')).href);await page.evaluate(()=>document.fonts.ready);
 const text=await page.locator('#resume').innerText();
 assert(text.includes('gxyhaoxianghenlihai@gmail.com'));
 for(const term of ['GPA：3.5','GPA：2.55','研发实习生','ROS2 架构师','2025.09—2026.08','负责 3 款','装配作业指导书','Copper-Aluminum Dissimilar Metals','Adaptive Dual-Super-Ellipsoid Sampling','尚未完成双臂与底盘整机联调','C1D'])assert(text.includes(term),term);
 for(const term of ['聚润','横向铰接','专升本','预计','ASR','MQTT','OTA'])assert(!text.includes(term),term);
 assert(text.indexOf('云深处')<text.indexOf('未来之城'));
 assert(await page.locator('.photo').evaluate(e=>e.complete&&e.naturalWidth>0));
 assert.equal(await page.locator('.page').count(),1);
 await page.emulateMedia({media:'print'});
 const geometry=await page.locator('.page').evaluate(e=>{
  const r=e.getBoundingClientRect();return {height:r.height,scroll:e.scrollHeight,last:e.lastElementChild.getBoundingClientRect().bottom-r.top,overflow:getComputedStyle(e).overflow};
 });
 console.log(JSON.stringify(geometry));
 assert(geometry.height<1124&&geometry.last<1090&&geometry.overflow==='visible');
 await page.pdf({path:path.join(out,'郭兴雨.pdf'),preferCSSPageSize:true,printBackground:true,displayHeaderFooter:false});
 await page.locator('.page').screenshot({path:path.join(__dirname,'HTML打印预览.png')});
 // Verify the existing HTML export retains editable mechanical content.
 await page.emulateMedia({media:'screen'});
 const dlPromise=page.waitForEvent('download');await page.locator('#saveHtml').click();
 const dl=await dlPromise,saved=fs.readFileSync(await dl.path(),'utf8');
 assert(saved.includes('负责 3 款'));assert(saved.includes('contenteditable="true"'));
 assert(!saved.includes("const storageKey='resume-mechanical-gxy-20260909-v04';"));
 assert.equal(errors.length,0);
 fs.writeFileSync(path.join(__dirname,'browser-check.json'),JSON.stringify({geometry,errors,checks:['content present','omissions retained','internship order','photo loaded','editable HTML export and isolated storage']},null,2));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});

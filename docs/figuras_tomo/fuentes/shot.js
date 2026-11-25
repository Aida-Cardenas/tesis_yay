const {chromium}=require('playwright');
(async()=>{
 const b=await chromium.launch({args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const views=process.argv.slice(2);
 for(const v of views){
  const p=await b.newPage({viewport:{width:1200,height:900},deviceScaleFactor:2});
  p.on('console',m=>console.log('console',m.text()));p.on('pageerror',e=>console.log('err',e.message));
  await p.goto(`http://127.0.0.1:8765/index.html?view=${v}&fill=0.97`);
  await p.waitForFunction(()=>window.__done===true,{timeout:120000});
  await p.screenshot({path:`out_${v}.png`});
  await p.close(); console.log('ok',v);
 }
 await b.close();
})();

const $=id=>document.getElementById(id);
const MOTORS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
let statusBusy=false, active=false, centering=null, presentMask=0;
let held=Array(15).fill(false), moved=Array(15).fill(false), dragging=null;
let gotoJob=null, gotoFlight=false;
let imuCalibrationActive=false;

async function post(path, body={}){
  const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const x=await r.json().catch(()=>({}));
  if(!r.ok) throw new Error(x.detail||'request failed');
  return x;
}
function showError(e){$('message').textContent=e.message||String(e);}

MOTORS.forEach((id,i)=>{
  $('joints').insertAdjacentHTML('beforeend',`<div class="cal-row" id="row${i}">
    <div class="cal-meta"><b>ID ${id}</b><span id="raw${i}">Waiting for feedback</span></div>
    <input id="sl${i}" type="range" min="0" max="4095" value="2047" aria-label="Position ID ${id}" disabled>
    <div class="cal-actions"><span id="val${i}">2047</span><span id="state${i}">Unloaded</span><button data-center="${i}" disabled>Set center &amp; hold</button></div>
  </div>`);
  const sl=$(`sl${i}`);
  sl.onpointerdown=()=>{dragging=i;};
  sl.oninput=()=>{
    moved[i]=true;
    $(`val${i}`).textContent=sl.value;
    if(active&&centering===null&&!held[i])queueGoto(i,+sl.value);
  };
  sl.onchange=()=>{if(dragging===i)dragging=null;};
});

function refreshRows(){
  document.querySelectorAll('[data-center]').forEach(btn=>{
    const i=+btn.dataset.center;
    const live=!!(presentMask&(1<<i));
    const busy=centering!==null||imuCalibrationActive;
    btn.disabled=!active||busy||held[i]||!live;
    $(`sl${i}`).disabled=!active||busy||held[i]||!live;
    $(`row${i}`).classList.toggle('done',held[i]);
    $(`state${i}`).textContent=centering===i?'Centering…':held[i]?'Holding center':'Unloaded';
  });
  $('start').disabled=imuCalibrationActive||centering!==null;
}
function queueGoto(i,pos){
  gotoJob={i,pos};
  if(!gotoFlight)pumpGoto();
}
async function pumpGoto(){
  if(gotoFlight||!gotoJob||centering!==null)return;
  const job=gotoJob;
  gotoJob=null;
  gotoFlight=true;
  try{await post('cal/goto',{index:job.i,raw_pos:job.pos});}
  catch(e){showError(e);}
  finally{gotoFlight=false;if(gotoJob)pumpGoto();}
}

document.querySelectorAll('[data-center]').forEach(btn=>{
  btn.onclick=async()=>{
    const i=+btn.dataset.center;
    centering=i;
    refreshRows();
    $('message').textContent=`Setting the center of ID ${MOTORS[i]}…`;
    try{
      await post('cal/center',{index:i});
      $('message').textContent=`ID ${MOTORS[i]} is holding its center.`;
    }catch(e){showError(e);}
    finally{centering=null;refreshRows();}
  };
});

async function enter(){
  if(imuCalibrationActive)return false;
  try{
    await post('cal/enter');
    active=true;
    held.fill(false);
    moved.fill(false);
    $('message').textContent='All servos are unloaded. Place a joint with its slider, then set its center.';
    return true;
  }catch(e){showError(e);return false;}
  finally{refreshRows();}
}
async function exit(){
  active=false;
  refreshRows();
  try{
    await post('cal/exit');
    held.fill(false);
    imuCalibrationActive=false;
    $('message').textContent='Calibration closed. All servos are unloaded.';
  }catch(e){showError(e);}
  refreshRows();
}
$('start').onclick=enter;
$('exit').onclick=exit;
$('exit2').onclick=exit;
window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/cal/exit',new Blob(['{}'],{type:'application/json'})));
document.addEventListener('visibilitychange',()=>{if(document.hidden&&active)exit();});

setInterval(async()=>{
  if(statusBusy||document.hidden)return;
  statusBusy=true;
  try{
    if(active)await post('cal/heartbeat').catch(()=>{});
    const r=await fetch('/api/status');
    if(!r.ok)throw new Error('status read failed');
    const s=await r.json();
    active=s.mode==='calibrate';
    presentMask=s.raw_mask||0;
    held=s.cal_done||Array(15).fill(false);
    imuCalibrationActive=!!s.imu_calibration?.active;
    $('mode').textContent=active?'Calibration active':'Calibration closed';
    if(s.last_error)$('message').textContent=s.last_error;
    if(s.raw_pos)s.raw_pos.forEach((v,i)=>{
      const live=!!(presentMask&(1<<i));
      $(`raw${i}`).textContent=live?'Encoder '+v:'No servo feedback';
      if(live&&!moved[i]&&dragging!==i&&v>=0&&v<=4095){
        $(`sl${i}`).value=v;
        $(`val${i}`).textContent=v;
      }
    });
    refreshRows();
  }catch(e){showError(e);}
  finally{statusBusy=false;}
},250);

enter();

import os,time
try: import psutil
except Exception: psutil=None
class SystemMonitor:
    def __init__(self): self.last_t=time.time(); self.last_net=self._net()
    def _temp(self):
        best=None
        for root,_,files in os.walk('/sys/class/thermal'):
            if 'temp' in files:
                try:
                    v=float(open(os.path.join(root,'temp')).read().strip()); v=v/1000 if v>1000 else v
                    if 0<v<120: best=v if best is None else max(best,v)
                except Exception: pass
        return best
    def _net(self):
        if not psutil: return None
        try:
            n=psutil.net_io_counters(); return (n.bytes_sent,n.bytes_recv)
        except Exception: return None
    def sample(self):
        now=time.time(); tx=rx=0.0; net=self._net()
        if net and self.last_net:
            dt=max(.001,now-self.last_t); tx=(net[0]-self.last_net[0])*8/1000/dt; rx=(net[1]-self.last_net[1])*8/1000/dt
            self.last_net=net; self.last_t=now
        if psutil:
            mem=psutil.virtual_memory(); disk=psutil.disk_usage('/')
            return {'cpu_percent':round(psutil.cpu_percent(),1),'cpu_per_core':[round(x,1) for x in psutil.cpu_percent(percpu=True)],'memory_percent':round(mem.percent,1),'memory_used_mb':round(mem.used/1024/1024,1),'memory_total_mb':round(mem.total/1024/1024,1),'disk_percent':round(disk.percent,1),'temperature_c':round(self._temp(),1) if self._temp() else None,'uptime_sec':round(now-psutil.boot_time(),1),'net_tx_kbps':round(tx,1),'net_rx_kbps':round(rx,1),'psutil_available':True}
        return {'cpu_percent':None,'cpu_per_core':[],'memory_percent':None,'disk_percent':None,'temperature_c':round(self._temp(),1) if self._temp() else None,'net_tx_kbps':round(tx,1),'net_rx_kbps':round(rx,1),'psutil_available':False}

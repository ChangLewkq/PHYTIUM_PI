import json,time,urllib.request
class RemoteMonitor:
    def __init__(self,url): self.base=(url or '').rstrip('/'); self.cache=None; self.ts=0
    def fetch_stats(self,min_interval=.25):
        now=time.time()
        if self.cache and now-self.ts<min_interval: return self.cache
        d={'online':False,'url':self.base,'video_feed':self.base+'/video_feed' if self.base else '', 'device':'unknown','model':'','fps':0,'inference_ms':0,'detect_count':0,'rgb_fps':0,'last_packet_age_sec':None,'message':'offline'}
        if self.base:
            try:
                with urllib.request.urlopen(self.base+'/api/stats',timeout=.45) as r: d=json.loads(r.read().decode()); d['online']=True; d['url']=self.base; d['video_feed']=self.base+'/video_feed'
            except Exception as e: d['message']=str(e)
        self.cache=d; self.ts=now; return d
    def switch_device(self,dev):
        try:
            req=urllib.request.Request(self.base+'/api/device',data=json.dumps({'device':dev}).encode(),method='POST',headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(req,timeout=3) as r: return json.loads(r.read().decode())
        except Exception as e: return {'ok':False,'message':str(e)}

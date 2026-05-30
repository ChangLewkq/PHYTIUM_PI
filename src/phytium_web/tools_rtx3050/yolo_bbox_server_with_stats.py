#!/usr/bin/env python3
# 简化版：在你现有 yolo_bbox_server.py 基础上，应至少增加 /api/stats 接口。
# 如果你需要完整替换版，下一步我可以按你当前 scripts_rtx3050/yolo_bbox_server.py 单独生成。
from flask import Flask, jsonify
app=Flask(__name__)
@app.route('/api/stats')
def stats(): return jsonify({'online':True,'device':'gpu','model':'yolov8.engine','fps':0.0,'rgb_fps':0.0,'inference_ms':0.0,'detect_count':0,'message':'replace this file with full server if needed'})
@app.route('/api/device',methods=['POST'])
def device(): return jsonify({'ok':True,'device':'gpu','message':'placeholder'})
if __name__=='__main__': app.run(host='0.0.0.0',port=8080,threaded=True)

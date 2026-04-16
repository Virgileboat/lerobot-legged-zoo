# Host: remote
- Type: local
- SSH:
- Remote dir: /home/vbatto/devel/lerobot-legged-zoo
- Tunnel:
- GPU check: nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits
- GPU threshold: 80
- PATH setup:
- Dependencies: uv sync

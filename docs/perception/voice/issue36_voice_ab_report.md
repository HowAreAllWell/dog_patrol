# Issue #36：源部署提交与 installed main 的历史 PCM A/B

- 验收日期：2026-08-06（Asia/Shanghai）
- 结论：**通过（迁入后无下降）**
- 范围：只验证冻结源部署提交与 clean-installed 主仓在同一历史 PCM、同一 Vosk 模型和同一有效识别参数下的迁移前后等价性；不作最终口令、FAR/FRR、实时性能或现场硬件结论。

## 结论摘要

| 层级 | 类别 | 源部署提交 | installed main | 结果 |
| --- | --- | ---: | ---: | --- |
| 任务级（33 runs） | 正例（20） | 18 通过 | 18 通过 | 一致 |
| 任务级（33 runs） | 负例（13） | 10 拒绝、3 误放行 | 10 拒绝、3 误放行 | 一致 |
| 窗口级（57 windows） | 正例窗口（32） | 18 通过 | 18 通过 | 一致 |
| 窗口级（57 windows） | 负例窗口（25） | 22 拒绝、3 误放行 | 22 拒绝、3 误放行 | 一致 |
| 技术错误 | 全部 | 0 | 0 | 一致 |

逐窗口比较得到 57/57 条记录的以下字段完全一致：最终决定、决定时间、输入字节数、实际回放时长、6 路文本、命中通道和 vote counts。源仓已有的 3 个负例误放行以及 2 个标为正例但源仓任务级拒绝的 run 均未被主仓新增或改变。

## 固定输入与版本

### 数据和模型

| 项目 | 固定值 |
| --- | --- |
| PCM 根目录 | 源仓 captures/r818-stream-acceptance-20260730/ |
| PCM 清单 | 57 files，218,239,040 bytes |
| PCM 清单聚合 SHA-256 | 18248932edb35672ab3b223d23d45cac9fddbe6ec9e259cf7a80fc7335d36dc6 |
| Vosk model | models/vosk-model-small-en-us-0.15/ |
| model 清单 | 14 files，70,898,967 bytes |
| model 清单聚合 SHA-256 | 5c9c563ec18e9a7d176eacdb18788b1c7dde7decf40c92ee2c8374668ade9656 |
| #32 清单 | docs/issue32_voice_deployment_manifest.sha256 |

PCM 和 model 不进入本仓提交；逐文件 SHA-256 仍由 #32 清单固定。下表中每个窗口显示该 PCM 文件的 SHA-256 前 12 位，完整值可由清单复核。

### 代码、配置和依赖

| 项目 | 源部署提交 | clean-installed main |
| --- | --- | --- |
| 代码版本 | moonshine_voice_commands@b979a7fd33aac5c9ced9591bb507e483faf4aef5，tree 2cc4e3fc0030ec8b3f618f6f3c624956889eec4d | dog_patrol@f6f683dd52b2d23c3dbd0a8da494e7c122055575 |
| 执行模块 | moonshine_voice_commands/src/moonshine_voice_commands/ | /tmp/dog_patrol_issue36_install_20260806/lib/python3.10/site-packages/dog_patrol_perception_voice/ |
| 配置文件 | config/commands.yaml，SHA-256 f940637b570fe0a3d3d89bc650854525887e5c0f31f9deec56b1a49d9e2445cb | 安装产物 share/dog_patrol_perception_voice/config/voice.yaml，SHA-256 889bd464c17b4c80052b599fff4ffa1178a91e6944cf07a46879ab9cff91767f |
| Python | 3.10.12 | 3.10.12 |
| Vosk | 0.3.45 | 0.3.45 |
| NumPy | 2.2.6 | 2.2.6 |
| PyYAML | 6.0.3 | 6.0.3 |

两个配置文件的 schema 不同，但本次识别有效字段归一后相同：passphrase blue star、grammar ["blue star", "[unk]"]、响应窗 20.0 秒、vote guard 0.75 秒；源仓配置没有 contrast phrase，故两侧 grammar 完全相同。两侧均只使用麦克风通道 0–5，通道 6–7 不送入 Vosk。

## 回放口径与命令

每个 run-N 作为一个固定样本；按文件名排序逐窗回放该 run 内全部 attempt-*-stream.pcm。每个 PCM 按 960 帧（60 ms、15,360 bytes）切块，按 20 秒窗口最多读取 5,120,000 bytes；任务级结果是该 run 已有窗口结果的 OR，仅用于比较两侧，不改变产品任务规则。两侧均使用独立进程，各自只加载一次相同 model。

源仓执行入口是 R818StreamingVoskSession；主仓执行入口是 clean-installed package 的 dog_patrol_perception_voice.r818_stream.R818StreamingVoskSession。回放使用两侧已经存在的 InterleavedPcmTaskStream.window_chunks() 注入 seam，不启动 ADB、Prompt、R818 或 ROS 节点。

构建 clean install：

~~~bash
set -eo pipefail
source_repo=/home/user/workspace/dog_patrol
main_commit=f6f683dd52b2d23c3dbd0a8da494e7c122055575
clean_worktree=/tmp/dog_patrol_issue36_worktree_20260806
test "$(git -C "$source_repo" rev-parse "$main_commit^{commit}")" = "$main_commit"
rm -rf "$clean_worktree"
git -C "$source_repo" worktree add --detach "$clean_worktree" "$main_commit"
cleanup_worktree() {
  git -C "$source_repo" worktree remove --force "$clean_worktree"
}
trap cleanup_worktree EXIT
cd "$clean_worktree"
source /opt/ros/humble/setup.bash
source "$source_repo/install/setup.bash"
set -u
rm -rf /tmp/dog_patrol_issue36_build_20260806 /tmp/dog_patrol_issue36_install_20260806
colcon build --packages-select dog_patrol_perception_voice \
  --build-base /tmp/dog_patrol_issue36_build_20260806 \
  --install-base /tmp/dog_patrol_issue36_install_20260806 \
  --merge-install --event-handlers console_direct+
~~~

源仓和 clean-installed main 使用同一个可复现的临时回放命令；它只通过现有
InterleavedPcmTaskStream seam 读 PCM，不是安装到产品包中的通用 evaluation/capture CLI：
回放完成或失败时，命令的 EXIT trap 会清理本次 build/install 临时前缀。

~~~bash
set -euo pipefail
cd /home/user/workspace/moonshine_voice_commands
test "$(git rev-parse HEAD)" = b979a7fd33aac5c9ced9591bb507e483faf4aef5
test -z "$(git status --porcelain)"
asset_root=/home/user/workspace/moonshine_voice_commands
captures="$asset_root/captures/r818-stream-acceptance-20260730"
model_dir="$asset_root/models/vosk-model-small-en-us-0.15"
manifest_sha256() {
  manifest_root="${1%/}"
  find "$manifest_root" -type f -print0 | LC_ALL=C sort -z |
    while IFS= read -r -d '' file; do
      relative_path="${file#"$manifest_root"/}"
      file_hash="$(sha256sum "$file" | cut -d' ' -f1)"
      printf '%s  %s\n' "$file_hash" "$relative_path"
    done | sha256sum | cut -d' ' -f1
}
test -f /home/user/workspace/dog_patrol/docs/issue32_voice_deployment_manifest.sha256
test "$(find "$captures" -type f | wc -l)" = 57
test "$(find "$captures" -type f -printf '%s\n' | awk '{ total += $1 } END { print total + 0 }')" = 218239040
test "$(manifest_sha256 "$captures")" = 18248932edb35672ab3b223d23d45cac9fddbe6ec9e259cf7a80fc7335d36dc6
test "$(find "$model_dir" -type f | wc -l)" = 14
test "$(find "$model_dir" -type f -printf '%s\n' | awk '{ total += $1 } END { print total + 0 }')" = 70898967
test "$(manifest_sha256 "$model_dir")" = 5c9c563ec18e9a7d176eacdb18788b1c7dde7decf40c92ee2c8374668ade9656
cleanup_outputs() {
  rm -rf /tmp/dog_patrol_issue36_build_20260806 /tmp/dog_patrol_issue36_install_20260806
}
trap cleanup_outputs EXIT
run_issue36_ab() {
  engine="$1"
  output="$2"
  install_prefix=/tmp/dog_patrol_issue36_install_20260806
  python_bin=/home/user/workspace/moonshine_voice_commands/.venv/bin/python
  if [ "$engine" = source ]; then
    env_args=(env -u PYTHONPATH)
  else
    env_args=(env -u PYTHONPATH)
  fi
  "${env_args[@]}" "$python_bin" -S - "$engine" \
    /home/user/workspace/moonshine_voice_commands \
    "$install_prefix" f6f683dd52b2d23c3dbd0a8da494e7c122055575 \
    > "$output" <<'PY'
import hashlib
import json
import sys
from collections import deque
from pathlib import Path

engine = sys.argv[1]
asset_root = Path(sys.argv[2])
install_prefix = Path(sys.argv[3])
main_commit = sys.argv[4]
captures = asset_root / "captures/r818-stream-acceptance-20260730"
model_dir = asset_root / "models/vosk-model-small-en-us-0.15"
venv_site = Path("/home/user/workspace/moonshine_voice_commands/.venv/lib/python3.10/site-packages")
sys.path.append(str(venv_site))
timeout_seconds = 20.0
sample_rate = 16_000
origin_channels = 8
mic_channels = 6
chunk_frames = 960
chunk_bytes = chunk_frames * origin_channels * 2
max_bytes = round(timeout_seconds * sample_rate * origin_channels * 2)

if engine == "source":
    sys.path.insert(0, str(asset_root / "src"))
    from moonshine_voice_commands.config import load_config
    from moonshine_voice_commands.r818_stream import R818StreamingVoskSession
    from moonshine_voice_commands.vosk_grammar import load_vosk_model
    config_path = asset_root / "config/commands.yaml"
    config = load_config(config_path)
    source_commit = "b979a7fd33aac5c9ced9591bb507e483faf4aef5"
    source_tree = "2cc4e3fc0030ec8b3f618f6f3c624956889eec4d"
else:
    install_site = (install_prefix / "lib/python3.10/site-packages").resolve()
    sys.path.insert(0, str(install_site))
    import dog_patrol_perception_voice
    from dog_patrol_perception_voice.config import load_voice_config
    from dog_patrol_perception_voice.r818_stream import R818StreamingVoskSession
    from dog_patrol_perception_voice.vosk import load_vosk_model
    config_path = install_prefix / "share/dog_patrol_perception_voice/config/voice.yaml"
    config = load_voice_config(config_path)
    module_path = Path(dog_patrol_perception_voice.__file__).resolve()
    if install_site not in module_path.parents:
        raise RuntimeError(f"installed module escaped clean install: {module_path}")
    source_commit = None
    source_tree = None

model, recognizer_factory = load_vosk_model(model_dir)

class ReplayStream:
    def __init__(self, paths):
        self._paths = deque(paths)

    def start(self):
        pass

    def close(self):
        pass

    def window_chunks(self, timeout):
        path = self._paths.popleft()
        limit = min(max_bytes, round(timeout * sample_rate * origin_channels * 2))

        def chunks():
            with path.open("rb") as stream:
                remaining = limit
                while remaining:
                    chunk = stream.read(min(chunk_bytes, remaining))
                    if not chunk:
                        break
                    remaining -= len(chunk)
                    yield chunk

        return chunks()

def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()

samples = []
for kind_dir in sorted(captures.iterdir()):
    if not kind_dir.is_dir():
        continue
    kind = "positive" if kind_dir.name.endswith("positive") else "negative"
    for run_dir in sorted(kind_dir.iterdir()):
        if not run_dir.is_dir():
            continue
        paths = sorted(run_dir.glob("attempt-*-stream.pcm"))
        stream = ReplayStream(paths)
        sample = {
            "sample_id": f"{kind_dir.name}/{run_dir.name}",
            "class": kind,
            "windows": [],
            "task_accepted": False,
            "technical_error": None,
        }
        try:
            if engine == "source":
                reports = []
                with R818StreamingVoskSession(
                    stream, config, model=model,
                    recognizer_factory=recognizer_factory,
                    vote_guard_seconds=0.75, on_report=reports.append,
                ) as session:
                    results = [
                        session.recognize("blue_star", timeout_seconds)
                        for _ in paths
                    ]
                details = [
                    {
                        "attempt_number": report.attempt_number,
                        "pcm_bytes": report.pcm_bytes,
                        "captured_duration_seconds": report.captured_duration_seconds,
                        "channel_results": list(report.channel_results),
                        "vote_counts": report.vote_counts,
                        "matching_channels": list(report.matching_channels),
                        "winning_phrase": report.winning_phrase,
                    }
                    for report in reports
                ]
            else:
                stream.start()
                session = R818StreamingVoskSession(
                    stream, config, model=model,
                    recognizer_factory=recognizer_factory,
                )
                try:
                    results = [
                        session.recognize(index, timeout_seconds)
                        for index, _ in enumerate(paths, start=1)
                    ]
                finally:
                    stream.close()
                details = [
                    {
                        "attempt_number": result.attempt_number,
                        "pcm_bytes": result.pcm_bytes,
                        "captured_duration_seconds": result.captured_duration_seconds,
                        "channel_results": list(result.channel_results),
                        "vote_counts": result.vote_counts,
                        "matching_channels": list(result.matching_channels),
                        "winning_phrase": (
                            config.passphrase if result.vote_counts else None
                        ),
                    }
                    for result in results
                ]
            for path, result, detail in zip(paths, results, details):
                sample["windows"].append({
                    "path": path.relative_to(captures).as_posix(),
                    "sha256": sha256(path),
                    "file_bytes": path.stat().st_size,
                    "result": {
                        "accepted": result.accepted,
                        "decision_time_seconds": result.decision_time_seconds,
                    },
                    "report": detail,
                })
            sample["task_accepted"] = any(
                window["result"]["accepted"] for window in sample["windows"]
            )
        except Exception as exc:
            sample["technical_error"] = f"{type(exc).__name__}: {exc}"
        samples.append(sample)

technical_errors = [sample for sample in samples if sample["technical_error"]]
window_count = sum(len(sample["windows"]) for sample in samples)
if len(samples) != 33 or window_count != 57 or technical_errors:
    raise RuntimeError(
        f"unexpected replay coverage: samples={len(samples)}, "
        f"windows={window_count}, technical_errors={technical_errors}"
    )

result = {
    "engine": "source-deployment-commit" if engine == "source"
        else "clean-installed-main",
    "source_commit": source_commit,
    "source_tree": source_tree,
    "main_commit": main_commit,
    "installed_module": (
        str(Path(dog_patrol_perception_voice.__file__).resolve())
        if engine != "source" else None
    ),
    "model": str(model_dir),
    "config": str(config_path),
    "timeout_seconds": timeout_seconds,
    "chunk_frames": chunk_frames,
    "chunk_bytes": chunk_bytes,
    "sample_rate": sample_rate,
    "origin_channels": origin_channels,
    "mic_channels": mic_channels,
    "vote_guard_seconds": 0.75,
    "samples": samples,
}
print(json.dumps(result, ensure_ascii=False))
PY
}
run_issue36_ab source /tmp/dog_patrol_issue36_source_20260806.json
run_issue36_ab installed /tmp/dog_patrol_issue36_installed_20260806.json
~~~

运行结果临时 artifact 的 SHA-256：

- source：87622c6090e1d6ad11f6e37f08420408e6aebf776ca928dd205b349546401433
- installed：cc4a81c22a4503b4e794c42bc5c788b09212c24b5340e50084cbc91bdab31391

检查项：clean-installed main 的 dog_patrol_perception_voice.__file__ 指向上述临时 install prefix；运行时未 import、source 或访问 dog_patrol/src 中的模块。结果文件只记录路径、校验值和识别结果，不含 PCM、model 或录音内容。

## 任务级结果

| 样本 | 类别 | 源 | installed main | 一致性 |
| --- | --- | --- | --- | --- |
| <code>blue-sky-negative/run-01</code> | 负例 | 通过 | 通过 | 一致 |
| <code>blue-sky-negative/run-02</code> | 负例 | 通过 | 通过 | 一致 |
| <code>blue-sky-negative/run-03</code> | 负例 | 通过 | 通过 | 一致 |
| <code>double-silence/run-01</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-02</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-03</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-04</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-05</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-06</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-07</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-08</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-09</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>double-silence/run-10</code> | 负例 | 拒绝 | 拒绝 | 一致 |
| <code>first-window-positive/run-01</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-02</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-03</code> | 正例 | 拒绝 | 拒绝 | 一致 |
| <code>first-window-positive/run-04</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-05</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-06</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-07</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-08</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-09</code> | 正例 | 通过 | 通过 | 一致 |
| <code>first-window-positive/run-10</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-01</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-02</code> | 正例 | 拒绝 | 拒绝 | 一致 |
| <code>second-window-positive/run-03</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-04</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-05</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-06</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-07</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-08</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-09</code> | 正例 | 通过 | 通过 | 一致 |
| <code>second-window-positive/run-10</code> | 正例 | 通过 | 通过 | 一致 |

## 逐窗口、逐通道结果

- file bytes / input bytes：原始 PCM 文件大小 / 在 20 秒 deadline 内实际送入 recognizer 的字节数。
- 通道顺序为 ch0; ch1; …; ch5；∅ 表示该通道没有完整 Vosk text。
- source result / installed result 都包含 PASS/REJECT 和决定时间；source matches / installed matches 为命中通道；vote counts 采用 phrase×count。
- 窗口的 SHA-256 在表中缩写为前 12 位，完整值见 #32 清单。

| PCM | SHA-256 | file/input bytes | duration (s) | source channels | installed channels | source result | installed result | matches source/installed | votes source/installed | comparison |
| --- | --- | --- | ---: | --- | --- | --- | --- | --- | --- | --- |
| <code>blue-sky-negative/run-01/attempt-1-stream.pcm</code> | <code>72f511635161…</code> | 5,120,000 / 1,474,560 | 5.760000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 5.730000s | PASS @ 5.730000s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>blue-sky-negative/run-01/attempt-2-stream.pcm</code> | <code>c761b9608c16…</code> | 5,120,016 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>blue-sky-negative/run-02/attempt-1-stream.pcm</code> | <code>df8e99a48889…</code> | 5,120,000 / 1,474,560 | 5.760000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 5.730000s | PASS @ 5.730000s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>blue-sky-negative/run-02/attempt-2-stream.pcm</code> | <code>3b249764a7cd…</code> | 5,120,016 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>blue-sky-negative/run-03/attempt-1-stream.pcm</code> | <code>8a25555a8344…</code> | 1,156,608 / 1,156,608 | 4.518000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 4.518000s | PASS @ 4.518000s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>double-silence/run-01/attempt-1-stream.pcm</code> | <code>d22a90711d0d…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-01/attempt-2-stream.pcm</code> | <code>71988849ed7e…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-02/attempt-1-stream.pcm</code> | <code>a20fb5ca22af…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-02/attempt-2-stream.pcm</code> | <code>9604e84100b5…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-03/attempt-1-stream.pcm</code> | <code>708b9ae0b2d8…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-03/attempt-2-stream.pcm</code> | <code>8d7195f64d56…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-04/attempt-1-stream.pcm</code> | <code>a44fa7bf73b2…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-04/attempt-2-stream.pcm</code> | <code>f070e702f17c…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-05/attempt-1-stream.pcm</code> | <code>1842468ef1f2…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-05/attempt-2-stream.pcm</code> | <code>bf3356185839…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-06/attempt-1-stream.pcm</code> | <code>6fa5afa17dd0…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-06/attempt-2-stream.pcm</code> | <code>04cd45faaa4a…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-07/attempt-1-stream.pcm</code> | <code>ec0d7882205d…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-07/attempt-2-stream.pcm</code> | <code>5b512088b556…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-08/attempt-1-stream.pcm</code> | <code>f23ba208626b…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-08/attempt-2-stream.pcm</code> | <code>4558b349be83…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-09/attempt-1-stream.pcm</code> | <code>8c4557a372e2…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-09/attempt-2-stream.pcm</code> | <code>2aa08e20e152…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-10/attempt-1-stream.pcm</code> | <code>39ba262adcd1…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>double-silence/run-10/attempt-2-stream.pcm</code> | <code>fe14d57f5911…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>first-window-positive/run-01/attempt-1-stream.pcm</code> | <code>d93a677d7b90…</code> | 787,968 / 787,968 | 3.078000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue; ch4=blue; ch5=blue | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue; ch4=blue; ch5=blue | PASS @ 3.078000s | PASS @ 3.078000s | 0,1,2 / 0,1,2 | blue star×3 / blue star×3 | 一致 |
| <code>first-window-positive/run-02/attempt-1-stream.pcm</code> | <code>f4e60bd2b19f…</code> | 5,120,000 / 1,290,240 | 5.040000 | ch0=∅; ch1=∅; ch2=blue star; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=blue star; ch3=∅; ch4=∅; ch5=∅ | PASS @ 5.010000s | PASS @ 5.010000s | 2 / 2 | blue star×1 / blue star×1 | 一致 |
| <code>first-window-positive/run-02/attempt-2-stream.pcm</code> | <code>2464d4da9530…</code> | 5,120,016 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>first-window-positive/run-03/attempt-1-stream.pcm</code> | <code>843d76f84997…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>first-window-positive/run-03/attempt-2-stream.pcm</code> | <code>40146d1b14a9…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>first-window-positive/run-04/attempt-1-stream.pcm</code> | <code>56e9424dc9c3…</code> | 1,527,056 / 1,527,056 | 5.965062 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 5.965062s | PASS @ 5.965062s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>first-window-positive/run-05/attempt-1-stream.pcm</code> | <code>f6d56dbe2f8a…</code> | 911,360 / 911,360 | 3.560000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=∅; ch5=[unk] | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=∅; ch5=[unk] | PASS @ 3.560000s | PASS @ 3.560000s | 0,1,2,3 / 0,1,2,3 | blue star×4 / blue star×4 | 一致 |
| <code>first-window-positive/run-06/attempt-1-stream.pcm</code> | <code>f3817a4286ec…</code> | 787,968 / 787,968 | 3.078000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 3.078000s | PASS @ 3.078000s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>first-window-positive/run-07/attempt-1-stream.pcm</code> | <code>e6bdb65290b7…</code> | 725,504 / 725,504 | 2.834000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=∅; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=∅; ch4=blue star; ch5=blue star | PASS @ 2.834000s | PASS @ 2.834000s | 0,1,2,4,5 / 0,1,2,4,5 | blue star×5 / blue star×5 | 一致 |
| <code>first-window-positive/run-08/attempt-1-stream.pcm</code> | <code>214756c25651…</code> | 787,968 / 787,968 | 3.078000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue [unk]; ch4=blue; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue [unk]; ch4=blue; ch5=blue star | PASS @ 3.078000s | PASS @ 3.078000s | 0,1,2,5 / 0,1,2,5 | blue star×4 / blue star×4 | 一致 |
| <code>first-window-positive/run-09/attempt-1-stream.pcm</code> | <code>64c400a7063a…</code> | 851,984 / 851,984 | 3.328063 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 3.328063s | PASS @ 3.328063s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>first-window-positive/run-10/attempt-1-stream.pcm</code> | <code>98bd1439b34f…</code> | 850,448 / 850,448 | 3.322063 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 3.322063s | PASS @ 3.322063s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>second-window-positive/run-01/attempt-1-stream.pcm</code> | <code>974c1b2bfd78…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-01/attempt-2-stream.pcm</code> | <code>e9fe835aa5ed…</code> | 973,824 / 973,824 | 3.804000 | ch0=blue; ch1=blue star; ch2=blue; ch3=[unk]; ch4=blue [unk]; ch5=[unk] | ch0=blue; ch1=blue star; ch2=blue; ch3=[unk]; ch4=blue [unk]; ch5=[unk] | PASS @ 3.804000s | PASS @ 3.804000s | 1 / 1 | blue star×1 / blue star×1 | 一致 |
| <code>second-window-positive/run-02/attempt-1-stream.pcm</code> | <code>04acf8f16990…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-02/attempt-2-stream.pcm</code> | <code>cde8e3d9a818…</code> | 5,119,984 / 5,119,984 | 19.999937 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-03/attempt-1-stream.pcm</code> | <code>0db4879399a7…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-03/attempt-2-stream.pcm</code> | <code>1b454cfbcd4d…</code> | 912,880 / 912,880 | 3.565938 | ch0=[unk]; ch1=star; ch2=star; ch3=[unk]; ch4=∅; ch5=blue star | ch0=[unk]; ch1=star; ch2=star; ch3=[unk]; ch4=∅; ch5=blue star | PASS @ 3.565938s | PASS @ 3.565938s | 5 / 5 | blue star×1 / blue star×1 | 一致 |
| <code>second-window-positive/run-04/attempt-1-stream.pcm</code> | <code>d70e5755ac67…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-04/attempt-2-stream.pcm</code> | <code>6d62b0cfac2e…</code> | 1,034,736 / 1,034,736 | 4.041938 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 4.041938s | PASS @ 4.041938s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>second-window-positive/run-05/attempt-1-stream.pcm</code> | <code>b73a4fa3d3eb…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-05/attempt-2-stream.pcm</code> | <code>6bd236533f88…</code> | 1,033,200 / 1,033,200 | 4.035938 | ch0=blue star; ch1=∅; ch2=blue star; ch3=blue star; ch4=blue; ch5=blue star | ch0=blue star; ch1=∅; ch2=blue star; ch3=blue star; ch4=blue; ch5=blue star | PASS @ 4.035938s | PASS @ 4.035938s | 0,2,3,5 / 0,2,3,5 | blue star×4 / blue star×4 | 一致 |
| <code>second-window-positive/run-06/attempt-1-stream.pcm</code> | <code>55cd52766433…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-06/attempt-2-stream.pcm</code> | <code>3cf8fcf33af6…</code> | 1,158,144 / 1,158,144 | 4.524000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue [unk]; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue [unk]; ch4=blue star; ch5=blue star | PASS @ 4.524000s | PASS @ 4.524000s | 0,1,2,4,5 / 0,1,2,4,5 | blue star×5 / blue star×5 | 一致 |
| <code>second-window-positive/run-07/attempt-1-stream.pcm</code> | <code>8fd973df4258…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-07/attempt-2-stream.pcm</code> | <code>fa07a236ed79…</code> | 1,466,112 / 1,466,112 | 5.727000 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 5.727000s | PASS @ 5.727000s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>second-window-positive/run-08/attempt-1-stream.pcm</code> | <code>7103666a7ed0…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-08/attempt-2-stream.pcm</code> | <code>4e2de5493494…</code> | 1,034,736 / 1,034,736 | 4.041938 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 4.041938s | PASS @ 4.041938s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>second-window-positive/run-09/attempt-1-stream.pcm</code> | <code>11f4e3b8e702…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-09/attempt-2-stream.pcm</code> | <code>a3cd19e751a1…</code> | 1,341,168 / 1,341,168 | 5.238937 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 5.238937s | PASS @ 5.238937s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |
| <code>second-window-positive/run-10/attempt-1-stream.pcm</code> | <code>7bc59bb0c5f6…</code> | 5,120,000 / 5,120,000 | 20.000000 | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | ch0=∅; ch1=∅; ch2=∅; ch3=∅; ch4=∅; ch5=∅ | REJECT @ 20.000000s | REJECT @ 20.000000s | — / — | — / — | 一致 |
| <code>second-window-positive/run-10/attempt-2-stream.pcm</code> | <code>8157efd3e050…</code> | 1,217,520 / 1,217,520 | 4.755937 | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | ch0=blue star; ch1=blue star; ch2=blue star; ch3=blue star; ch4=blue star; ch5=blue star | PASS @ 4.755937s | PASS @ 4.755937s | 0,1,2,3,4,5 / 0,1,2,3,4,5 | blue star×6 / blue star×6 | 一致 |

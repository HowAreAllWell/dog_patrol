# dog_patrol #4：tracking 迁移基线审计

- 审计日期：2026-08-04（Asia/Shanghai）
- 源仓：`HowAreAllWell/vision_demo_ws`
- 目标仓：`HowAreAllWell/dog_patrol`
- 对应问题：[dog_patrol #4](https://github.com/HowAreAllWell/dog_patrol/issues/4)
- 父规格：[dog_patrol #3](https://github.com/HowAreAllWell/dog_patrol/issues/3)
- 信息处理：疑似凭据、设备端点和隐私值不写入本文，只记录类别、路径、提交和处置。

## 结论

迁移基线为 `vision_demo_ws` 提交
`380b44582c0c55e5e46d2eb862da7700f05349b3`，tree 为
`dee104ceec9ffd00a579b312fc6ee3272602cdf3`。源仓 `dev`、本地及远端
`deploy/dog_patrol-integration` 均指向该提交；冻结时非忽略工作树干净。

该基线能够构建并通过既有测试，但不能把整个仓库或整个 package 历史直接导入：

- Git 历史没有大二进制对象，超过 512 KiB 的 blob 全是明确排除的 `worklog.md` 历史；
- 当前忽略目录仍有约 1.70 GiB 构建产物、日志和模型资产；
- 早期历史包含 RTSP userinfo 模板、私网端点、本机绝对路径，以及已退役的 RTSP、UDP、bearing 和录制路径；
- 源 package 声明 `Apache-2.0`，但源仓没有许可证全文、NOTICE 或源码 header；目标 BSD 主仓必须保留 Apache 组件归属并补齐发行材料。

后续历史导入必须使用本文白名单，在临时 clone 中清洗后复扫；本票不执行历史导入，也不改变 tracking、identity、mission 或输出行为。

## 1. 冻结起点与工作区处置

### 1.1 回退前清单

本次冻结前保存的 `git status --short --branch` 与 `git clean -nd` 记录精确列出：

| 状态 | 路径 | 处置 |
|---|---|---|
| modified | `worklog.md` | 用户明确要求回退到最新 Git 提交后恢复为 `HEAD` |
| untracked | `.agents/` | 同上，删除 |
| untracked | `.codex/` | 同上，删除 |
| untracked | `.pi-subagents/` | 同上，删除 |
| untracked | `docs/pi-auto-compaction-research.md` | 同上，删除 |
| untracked | `docs/prds/` | 同上，删除 |
| untracked | `docs/research/` | 同上，删除 |

用户在 #4 工作过程中明确指示“直接回退到本仓最新的 git 提交”，因此上述内容是本票唯一获授权丢弃的本机状态。执行了 `git reset --hard HEAD` 和经 dry-run 确认的 `git clean -fd`；忽略内容未删除。

### 1.2 冻结后的可复现证据

```bash
cd /home/user/workspace/vision_demo_ws
git status --porcelain=v2 --branch
git show -s --format='%H%n%T%n%aI%n%s' HEAD
git for-each-ref --format='%(refname) %(objectname)' \
  refs/heads/dev refs/remotes/origin/dev \
  refs/heads/deploy/dog_patrol-integration \
  refs/remotes/origin/deploy/dog_patrol-integration
```

结果：

- 非忽略 modified、staged、untracked 项为 0；
- `dev` 与 `origin/dev` ahead/behind 为 `0/0`；
- 冻结提交时间 `2026-08-03T16:37:05+08:00`，主题“config: 抽取 perception 配置物化模块”；
- `deploy/dog_patrol-integration` 已推送，起点与上述完整提交标识一致。

以下忽略内容仍在本机，仅用于分类证据，全部排除：

| 路径 | 文件数 | 大小（字节） | 类别 |
|---|---:|---:|---|
| `.pytest_cache/` | 5 | 12,818 | 测试缓存 |
| `MvSdkLog/` | 6 | 1,612,267 | 设备日志 |
| `assets/models/engines/` | 1 | 8,373,368 | TensorRT engine |
| `assets/models/reid/` | 3 | 18,217,663 | ONNX/权重数据 |
| `assets/models/upstream/` | 1 | 5,548,549 | 上游模型权重 |
| `build/` | 2,228 | 295,256,267 | 构建产物 |
| `install/` | 87 | 44,042,300 | 安装产物 |
| `log/` | 9,096 | 1,329,282,450 | colcon/测试日志 |

## 2. 迁入、提炼和排除清单

### 2.1 迁入白名单

仅保留下列路径及其相关历史；重命名和目标路径由后续迁移票处理：

| 源路径 | 基线文件数 | 处置 |
|---|---:|---|
| `src/vision_demo_host/CMakeLists.txt`、`package.xml` | 2 | 构建及 package 元数据；保持 `Apache-2.0` |
| `src/vision_demo_host/include/**` | 30 | C++ 接口 |
| `src/vision_demo_host/src/**` | 93 | runtime、算法、ROS adapter、录制及离线工具 |
| `src/vision_demo_host/test/**` | 54 | 行为与集成回归 |
| `src/vision_demo_host/config/**` | 12 | 默认、诊断和 legacy 对照；绝对路径参数化 |
| `src/vision_demo_host/launch/**` | 1 | 启动入口，后续随 package 改名 |
| `src/vision_demo_host/scripts/{bench_hik_mvs_camera,check_orin_env,eval_tracker_core_round1,export_yolo26n_engine_orin_jp621}.sh` | 4 | 当前验收、环境检查和 engine 本机构建工具 |
| `src/vision_demo_host/README.md` | 1 | 稳定说明来源；移除旧仓入口和本机路径 |
| `assets/models/manifests/vision-demo-tools_core_requirements.txt` | 1 | 轻量依赖说明，不含模型 |
| `docs/current_tracking_identity_state.md`、`docs/tracking_algorithm_design_and_tuning.md` | 2 | 当前状态和算法语义 |
| `docs/issue{80,81,82,83,85,86,87,92,93}_*.md` | 9 | 当前合同、硬件、媒体和 mission 验收证据；迁入时参数化路径 |

`src/vision_demo_host/**` 基线共 197 个文件，相关历史覆盖 111 个提交，起于
`161b56dcf5fe19bb563730418b0fe950aaaf7f8f`，止于冻结提交。

### 2.2 只提炼事实

- `assets/models/manifests/yolo26n_engine_build.md`：只提炼版本、模型来源、校验和和构建步骤；不复制本机路径或私网信息。
- `docs/orin_jp621_mvcu013_deployment.md`：只提炼已验证平台、SDK 和设备模式。
- `docs/orin_hik_h264_MOT_01_02_issue_resolution.md`：只保留仍适用于当前 tracking 的结论。
- `docs/phase5_birth_hidden_candidate_readiness.md`：把仍有效的不变量并入 tracking 设计说明。

### 2.3 明确排除

- 仓库管理和个人状态：`.github/**`、`AGENTS.md`、`docs/agents/**`、`worklog.md`；
- 旧仓入口和过程资料：根 `README.md`、`ORIN_PACKAGE_CONTENTS.md`、`docs/tracking_identity_refactor_plan.md`、历史 `docs/prds/**`；
- 构建、本机和 agent 状态：`build/**`、`install/**`、`log/**`、`.pytest_cache/**`、编辑器及 agent 工具目录；
- 数据、媒体和隐私材料：`data/**`、录像、相机日志、CSV/评估输出、人脸白名单和特征向量；
- 二进制资产：`*.engine`、`*.onnx`、`*.onnx.data`、`*.pt`、`*.pth`；
- HEAD 已退役的 bearing、UDP、RTSP benchmark、`record_test_set` 和对应测试；
- HEAD 已删除的 `legacy_identity_*` 目标树内容；只有保留文件的逐文件 ancestry 确实依赖时才保留相关提交。

## 3. Git 历史对象和敏感信息审计

### 3.1 对象审计

```bash
git rev-list --objects --all |
  git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' |
  awk '$1=="blob"' | sort -k3nr

git log --all --pretty=format: --name-only |
  sed '/^$/d' | sort -u |
  rg -i '\.(mp4|mkv|avi|mov|engine|onnx|pt|pth|bin|zip|tar|gz|bag|db3|jpg|jpeg|png|bmp|pcap)$'
```

结果：完整 refs 有 163 个提交、1,132 个唯一 blob；没有 ≥1 MiB blob。≥512 KiB 的 22 个 blob 全是明确排除的 `worklog.md` 历史，最大 566,337 字节。排除 worklog 后，最大历史 blob 是已删除的 `legacy_identity_matcher.cpp`，89,258 字节。历史路径没有模型、录像、图片、ROS bag 或压缩包扩展名。

### 3.2 敏感信息审计

对全部 refs 运行仅输出 commit/path、不输出匹配值的规则扫描，覆盖私钥头、常见云和 Git token、password/secret/token/key 赋值、URL userinfo、SSH key、MAC、设备 serial、私网 IPv4、本机绝对路径、模型/录像和生物特征术语。

| 类别 | 结果 | 处置 |
|---|---|---|
| 私钥、常见云/Git token、SSH key、MAC | 未命中 | 导入后复扫 |
| token 赋值 | 两处业务变量误报，不是凭据 | 保留 |
| RTSP URL userinfo | 历史 `demo_params.yaml`、package README、已删除 `bench_gmc_rtsp.sh` 命中占位符或环境变量形式；未发现字面凭据 | 排除退役路径；保留历史中的模板替换为无凭据参数说明 |
| 私网 IPv4/设备端点 | 早期历史命中；HEAD 的模型构建说明仍有私网信息 | 不原样迁入说明；保留代码历史中的端点改为占位符 |
| 本机绝对路径 | 配置、工具默认值、脚本、README 和若干验收文档命中 | 改为 package share、参数、环境变量或测试临时目录 |
| 设备 serial | 人工复核仅为枚举/变量代码，未见硬编码库存 serial | 保留代码；部署 serial 留在本机配置 |
| 模型、录像、数据集 | Git 历史未见二进制；忽略目录有 5 个模型/engine 文件 | 只迁轻量 manifest |
| 人脸白名单/生物特征 | 全历史未命中 | 后续继续隔离 |

本机未安装 `gitleaks` 或 `trufflehog`，所以当前是确定性规则扫描而非完整高熵扫描。导入门禁为：

1. 在临时 clone 中按白名单过滤，不改写源仓；
2. 替换私网端点、RTSP userinfo 模板和 `/home/<user>/...`，且替换表不进入日志或 PR；
3. 验证目标树没有排除路径和扩展名；
4. 对新 commit 图重跑对象、路径和敏感信息扫描；工具可用时补跑 `gitleaks git` 或等价扫描；
5. 复跑同一行为测试，禁止借清洗改变输出语义。

## 4. 许可证来源与保留方式

仓库事实：

- `src/vision_demo_host/package.xml` 自初始提交起声明 `<license>Apache-2.0</license>`，冻结提交仍相同；
- 源仓完整历史没有 `LICENSE`、`NOTICE`、`COPYING`、SPDX header 或版权归属声明；
- dog_patrol 根 `LICENSE` 和现有 package 为 `BSD-3-Clause`；
- 未发现 vendored 第三方源码；模型文件不迁入，其许可证不随代码迁移。

Apache-2.0 第 4 节要求再分发时提供许可证副本、显著声明修改，并保留相关版权、专利、商标和归属声明；上游存在 NOTICE 时还需保留相关内容。来源：[Apache License 2.0 原文](https://www.apache.org/licenses/LICENSE-2.0.txt)、[Apache 官方应用指南](https://www.apache.org/legal/apply-license)、[SPDX Apache-2.0](https://spdx.org/licenses/Apache-2.0.html)、[SPDX BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html)。

迁入 dog_patrol 时必须：

1. 根 BSD-3-Clause 保持不变，但明确其不覆盖另行标注的 Apache 组件；
2. 新增 `LICENSES/Apache-2.0.txt` 官方全文和组件许可证映射，明确 tracking 源码及随迁文档范围；
3. tracking package 的 `package.xml` 继续标记 `Apache-2.0`，不得直接改为纯 BSD；
4. 保留后续发现的原版权和 attribution；源仓无 NOTICE，不虚构 NOTICE；
5. 路径、package、namespace 或源码修改后显著标明已修改，并采用权利人确认的 header/SPDX 标识；
6. 公开发布前由权利人确认 package 声明确实覆盖全部迁入源码和文档。当前缺少全文/header 是待补的合规缺口，本文不构成法律意见。

## 5. 迁移改造前验证

```bash
cd /home/user/workspace/vision_demo_ws
source /opt/ros/humble/setup.bash
source /home/user/workspace/dog_patrol/install/setup.bash
colcon build --packages-select vision_demo_host --event-handlers console_direct+
colcon test --packages-select vision_demo_host \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose --all
```

结果：构建成功；53/53 CTest 通过；测试结果汇总 392 tests、0 errors、0 failures、0 skipped。未运行真实 TensorRT engine、Hik 相机或现场性能验收；这些需要受控硬件和本机资产，不影响本票“迁移前既有测试”验收。

## 6. 验收结论

- 回退前 modified/untracked 内容已逐项记录，且仅处置用户明确授权丢弃的内容；
- 临时部署分支从已确认干净提交创建并推送，起点和 tree 均已记录；
- 迁入白名单、只提炼范围和排除清单已固定；
- 大对象、凭据、设备信息、隐私和退役历史已检查并建立导入门禁；
- Apache-2.0 来源、当前合规缺口及在 BSD 主仓中的保留方式已固定；
- 冻结基线构建及既有测试全绿；
- 本票只新增审计证据和状态记录，未修改源码、配置或运行行为。

下一步是在后续迁移票中用临时 clone 生成过滤历史，补齐 Apache 发行文件，复扫后再导入 dog_patrol。

# PR 提交流程与自动化治理

## 目标

仓库当前的 PR 治理目标很明确：

- PR 页面只保留一个统一 workflow 入口：`.github/workflows/pr-ci.yml`
- Branch protection 只依赖一个稳定门禁：`PR gate`
- 失败时在 PR 中维护一条固定机器人汇总评论
- 检查通过后自动启用 squash auto-merge，尽量减少人工点选
- 创建 PR 时自动填充统一模板，降低描述缺失和格式不一致问题

## 标准 PR 提交流程

### 1. 基于最新 `master` 创建分支

建议流程：

```bash
git fetch origin master
git switch -c <type>/<short-desc> origin/master
```

建议分支命名保持单一目的，例如：

- `feat/add-delta-arm-monitor`
- `fix/handle-rosdep-reinit`
- `docs/update-pr-guide`
- `ci/unify-pr-automation`

### 2. 控制改动范围，并完成最小本地验证

每个 PR 应尽量只做一件事，避免把功能、重构、格式调整、无关修复混在一起。

提交前至少完成与本次改动对应的最小验证，例如：

- 文档改动：检查链接、标题、示例命令是否正确
- 配置改动：检查 YAML / 参数一致性
- 代码改动：按需执行 `colcon build`、`colcon test` 或关键运行路径验证

### 3. 使用语义化 PR 标题

统一要求：

- 格式：`<type>: <description>`
- 允许类型：`feat`、`fix`、`docs`、`refactor`、`test`、`chore`、`ci`

示例：

- `feat: add chassis command timeout handling`
- `fix: handle rosdep init reuse in CI`
- `docs: clarify PR gate branch protection setup`
- `ci: unify PR automation workflow`

标题是自动化检查的一部分，标题不符合规范时，`Semantic PR title` 会直接失败。

### 4. 创建 PR，并使用默认自动填充模板

仓库已经配置默认 PR 模板：

- `.github/pull_request_template.md`

创建 PR 时，GitHub 会自动填充该模板。作者需要按模板补全：

- 改动目的 / 背景
- 关联 Issue
- 变更类型
- 本地验证情况
- 关键命令与结果
- 额外影响说明（如配置、接口、兼容性）

### 5. 等待统一 workflow 自动处理

统一 PR workflow 会执行以下 job：

- `Semantic PR title`
- `ROS build and test`
- `PR gate`
- `PR automation`

其中：

- `PR gate` 聚合前两项检查结果，是 branch protection 唯一需要勾选的 required check
- `PR automation` 负责更新固定机器人评论，并在检查通过后启用 squash auto-merge

### 6. 自动合并行为

当前仓库已经验证可行的自动合并路径是：

- PR 不是 Draft
- `Semantic PR title` 通过
- `ROS build and test` 通过
- `PR gate` 通过
- 不存在分支冲突
- Branch protection 不再强制 approvals

满足这些条件后，GitHub 会自动启用 squash auto-merge，并在条件满足时自动合并到 `master`。

## 自动化行为说明

### `Semantic PR title`

负责校验 PR 标题是否符合语义化命名规则。

### `ROS build and test`

复用现有稳定检查：

- 工作区 `src/` 目录校验
- ROS 依赖安装
- `colcon build`
- `colcon test`
- `colcon test-result --verbose`

### `PR gate`

这是聚合门禁 job：

- 依赖 `Semantic PR title` 和 `ROS build and test`
- 只要其中任一失败，`PR gate` 就失败
- Branch protection 只需要勾选这一项

### `PR automation`

负责：

- 在 PR 中维护一条固定机器人汇总评论
- 展示标题检查、构建测试、`PR gate` 的结果
- 显示自动化处理结果
- 在条件满足时启用 squash auto-merge

补充说明：

- 当前 GitHub Actions 可能不被允许直接 approve PR
- 因此评论里可能出现“自动审批失败”的提示
- 但在当前仓库设置下，只要不强制 approvals，这不会阻塞自动 squash 合并

## 固定机器人评论

PR 中会维护一条固定机器人评论，而不是每次新发一条。

评论会汇总：

- `Semantic PR title` 结果
- `ROS build and test` 结果
- `PR gate` 结果
- 失败 job 列表
- 自动化处理状态
- 关联 workflow run 链接

这条评论是排查失败原因的第一入口。

## Branch protection 推荐配置

在 `Settings -> General` 中确认：

- 开启 `Allow auto-merge`

在 `Settings -> Branches -> Branch protection rules` 中建议：

- Branch name pattern：`master`（或你的默认分支）
- 开启 `Require a pull request before merging`
- 开启 `Require status checks to pass before merging`
- 必需检查只勾选：`PR gate`
- 建议开启 `Require branches to be up to date before merging`

如果目标是“检查通过后自动合并”，则不建议继续开启：

- 强制人工 approvals
- 强制 Code Owner approvals

原因很简单：当前自动化主路径依赖 `PR gate` + auto-merge，而不是 bot approval。

## CODEOWNERS 的使用建议

保留 `.github/CODEOWNERS` 的价值仍然存在：

- reviewer 请求提示
- ownership 声明
- 核心目录责任边界表达

但推荐把它作为 reviewer 提示机制，而不是自动合并的强制门禁。

## fork PR 说明

fork PR 与同仓库 PR 的共同点：

- 都会执行 `Semantic PR title`
- 都会执行 `ROS build and test`
- 都会生成 / 更新固定机器人评论
- 都会尝试启用 auto-merge

但 fork PR 的特权操作是否都能生效，仍受 GitHub 权限策略影响。

因此对 fork PR 应按以下原则理解：

- 检查结果可以信任
- 评论更新通常可以成功
- 自动化合并能力以仓库当前权限配置和 GitHub 平台限制为准
- 若 PR 修改了 `.github/workflows/*`，启用 auto-merge 还需要 `workflows: write` 权限；否则会在 `PR automation` 中提示需维护者手动启用

### 常见卡点：`PR gate` 一直显示“等待状态报告”，且没有 Run 按钮

出现这个现象时，通常不是 `pr-ci.yml` 逻辑错误，而是 **workflow 根本没有被触发**。  
典型信号是：

- PR 页面显示 `PR gate` 为 Expected / Pending
- 对应 commit 的 checks 数量为 0（没有任何 `Semantic PR title` / `ROS build and test` / `PR gate` 运行记录）
- PR 页面看不到可手动触发的 Run 入口

建议按下面顺序排查仓库设置（需要仓库管理员权限）：

0. 先确认触发时序
   - GitHub Actions 设置变更通常**不会追溯修复旧事件**
   - 如果 PR 在“受限配置”下创建，后续即使改好了设置，也可能仍停留在 pending
   - 需要重新触发事件：`close/reopen PR` 或向 PR 分支 `push` 一个新 commit（可为空提交）
1. `Settings -> Actions -> General`
   - 确认仓库允许执行 GitHub Actions workflow
   - 确认 fork PR 场景未被策略直接阻断（例如要求外部贡献者先人工批准才允许运行）
2. `Settings -> Actions -> General -> Workflow permissions`
   - 选择 `Read and write permissions`
   - 开启 `Allow GitHub Actions to create and approve pull requests`
3. `Settings -> Branches -> Branch protection rules`
   - required checks 保持与实际会运行的检查一致（建议只要求 `PR gate`）
   - 规则分支模式必须匹配真实目标分支（`master` / `main`），避免规则误配

如果你是 fork PR 作者而不是目标仓库管理员，通常看不到“批准并运行 workflow”的入口；需要由目标仓库维护者在仓库设置或 PR 页面执行一次批准。

## PR 模板填写规范

默认自动填充模板应至少完整填写以下内容：

1. **Summary**
   - 说明为什么改
   - 说明改了什么
   - 必要时说明影响范围

2. **Related issue**
   - 若有 Issue，使用 `Closes #123` / `Refs #123`

3. **Change type**
   - 与标题类型保持一致

4. **Validation**
   - 写清实际执行过的验证
   - 不要只打勾，不写命令和结果

5. **Additional notes**
   - 若涉及配置、接口、兼容性、发布注意事项，应明确写出

## 推荐自检清单

提交 PR 前，至少自检以下事项：

- [ ] 标题符合 `<type>: <description>`
- [ ] 改动范围单一且明确
- [ ] 本地最小验证已完成
- [ ] 文档 / 配置已同步更新（如适用）
- [ ] 未提交编译产物、缓存、临时文件
- [ ] PR 描述中的验证记录可被 reviewer 直接复现

## 已完成的 smoke test 结论

当前流程已经通过最小 docs PR 验证：

- `PR gate` 可作为唯一 required check
- 固定机器人评论正常更新
- squash auto-merge 能自动启用并完成合并
- 在关闭强制 approvals 后，流程可自动合并到主分支

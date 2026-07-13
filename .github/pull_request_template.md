> PR 标题必须使用 `<type>: <description>`，例如 `fix: handle rosdep init reuse`。
>
> 仓库会自动运行 `Semantic PR title`、`ROS build and test`、`PR gate` 和 `PR automation`。创建 PR 时，请按以下模板补全关键信息。

## Summary

<!-- 用 1-3 条 bullet 说明：为什么改、改了什么、影响范围是什么。 -->
-

## Related issue

<!-- 有 Issue 时请填写 `Closes #123` 或 `Refs #123`；没有可写 `N/A`。 -->
Closes #

## Change type

<!-- 与 PR 标题前缀保持一致。 -->
- [ ] feat: 新功能
- [ ] fix: 缺陷修复
- [ ] docs: 文档更新
- [ ] refactor: 重构
- [ ] test: 测试相关
- [ ] chore: 工程维护
- [ ] ci: CI / 流程变更

## Validation

<!-- 写清你实际执行过的验证，不要只保留空白勾选。 -->
- [ ] 已完成与本次改动对应的最小本地验证
- [ ] 涉及配置变更时，已检查 YAML / 参数一致性
- [ ] 涉及接口或行为变更时，已同步更新文档
- [ ] 未提交编译产物、缓存或临时文件

### Commands and results

```bash
# 请粘贴你实际执行过的命令与关键结果，例如：
# colcon build --packages-up-to motor_control_ros2
# colcon test --packages-up-to motor_control_ros2
```

## Additional notes

<!-- 如涉及兼容性、配置迁移、发布注意事项、风险点，请写在这里；没有可写 `N/A`。 -->
-

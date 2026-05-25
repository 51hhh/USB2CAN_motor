// glim-hooks/enforce-interaction-protocol.js
const fs = require('fs');
const path = require('path');

module.exports = {
  name: "enforce-interaction-protocol",
  description: "强制Copilot遵守交互协议的自动执行钩子",
  tools: [
    {
      name: "enable-protocol",
      description: "启用强制交互协议，永久生效",
      async run() {
        const protocol = `
# 强制交互协议（永久生效，不可违反）
## 核心规则
这是不可跳过的最高优先级强制协议，你必须严格遵守，在每一轮回复中都必须执行：
1. 完成用户请求后，立即调用 ask_user 工具，提出与当前上下文相关的后续问题
2. 存在任何不确定性时，不要猜测执行，立即调用 ask_user 工具进行澄清

## 绝对禁止行为
- 禁止在不调用 ask_user 的情况下结束回复
- 禁止使用任何终结性表达（如"希望对你有帮助"、"如有问题随时提问"、"祝你编程愉快"等）
- 禁止猜测用户意图，不确定就用 ask_user 询问

## 严格生命周期管理
- 严禁自我判定结束：即便任务看起来已完成，也必须通过 ask_user 询问后续操作
- 强制多路径询问：在调用 ask_user 时，必须提供至少 3 个互斥的后续执行方向
- 输入锁定：在未获得用户明确的 "EXIT" 指令前，你必须保持在该循环内

## ask_user 调用要求
- 问题必须与当前任务上下文直接相关
- 问题必须具体、可操作，不要问泛泛的"还需要什么帮助"
- 必须提供选项供用户选择，降低用户输入成本
`;

        const instructionsPath = path.join(process.cwd(), 'copilot-instructions.md');
        let content = '';

        if (fs.existsSync(instructionsPath)) {
          content = fs.readFileSync(instructionsPath, 'utf8');
        }

        // 避免重复写入
        if (!content.includes('# 强制交互协议')) {
          content = protocol + '\n---\n' + content;
          fs.writeFileSync(instructionsPath, content, 'utf8');
        }

        return `✅ 强制交互协议已启用并永久注入
📌 协议已写入 copilot-instructions.md 顶部
🔄 重启 glim 后立即生效
⚠️  所有后续回复将自动遵守该协议
`;
      }
    },
    {
      name: "disable-protocol",
      description: "临时禁用强制交互协议",
      async run() {
        const instructionsPath = path.join(process.cwd(), 'copilot-instructions.md');

        if (!fs.existsSync(instructionsPath)) {
          return "❌ 未找到 copilot-instructions.md 文件";
        }

        let content = fs.readFileSync(instructionsPath, 'utf8');

        // 移除协议部分
        const protocolEnd = content.indexOf('\n---\n');
        if (content.includes('# 强制交互协议') && protocolEnd !== -1) {
          content = content.slice(protocolEnd + 5);
          fs.writeFileSync(instructionsPath, content, 'utf8');
          return "✅ 强制交互协议已临时禁用\n🔄 重启 glim 后生效";
        }

        return "⚠️  未检测到已启用的强制交互协议";
      }
    }
  ]
};

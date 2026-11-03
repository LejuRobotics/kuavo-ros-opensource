#pragma once
namespace wheel_arm_abs_vel {
// Controller input policy shared by the optional V9 adapter and its regression.
// Completion is acknowledged by the executing IK consumer, never by a key press.
struct DirectTcpStartup {
  enum Action { None, Start, ToggleArmHold };
  bool completed=false, trigger_down=false, xa_down=false, xb_down=false;
  Action update(bool triggers, bool xa, bool xb, bool inactive, bool active, bool healthy) {
    Action action=None;
    if (xb && !xb_down && active && healthy) action=ToggleArmHold;
    else if (inactive && !xb && ((!completed && triggers && !trigger_down)
             || (completed && xa && !xa_down))) action=Start;
    trigger_down=triggers;xa_down=xa;xb_down=xb;
    return action;
  }
};
}

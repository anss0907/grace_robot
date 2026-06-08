# Charger adapter relay state — controls whether each charger adapter is powered
# active = true means the relay is energised and the charger adapter is receiving mains power

std_msgs/Header header

bool charger_40v_on    # True = 40V charger adapter relay energised (powered ON)
bool charger_24v_on    # True = 24V charger adapter relay energised (powered ON)

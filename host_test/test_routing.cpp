#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "../mesh_node/routing.h"
#include "../mesh_node/link_health.h"

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s\n", msg); \
    exit(1); \
  } else { \
    printf("ok   : %s\n", msg); \
  } \
} while (0)

#define CHECK_NEAR(val, exp, msg) do { \
  if (std::fabs((val) - (exp)) >= 0.001f) { \
    printf("FAIL: %s (got %.4f, expected %.4f)\n", msg, (float)(val), (float)(exp)); \
    exit(1); \
  } else { \
    printf("ok   : %s (%.2f)\n", msg, (float)(val)); \
  } \
} while (0)

int main() {
  // ---- 1. Link Penalty Formula Tests: penalty = COST_SCALE * (1.0f - q_pred) ----
  LinkStats s;
  s.qPred = 1.00f; CHECK_NEAR(link_penalty(s), 0.00f, "penalty: q_pred 1.00 -> 0.00");
  s.qPred = 0.90f; CHECK_NEAR(link_penalty(s), 1.00f, "penalty: q_pred 0.90 -> 1.00");
  s.qPred = 0.75f; CHECK_NEAR(link_penalty(s), 2.50f, "penalty: q_pred 0.75 -> 2.50");
  s.qPred = 0.60f; CHECK_NEAR(link_penalty(s), 4.00f, "penalty: q_pred 0.60 -> 4.00");
  s.qPred = 0.50f; CHECK_NEAR(link_penalty(s), 5.00f, "penalty: q_pred 0.50 -> 5.00");
  s.qPred = 0.30f; CHECK_NEAR(link_penalty(s), 7.00f, "penalty: q_pred 0.30 -> 7.00");

  // ---- 2. Basic Routing & Priority Override ----
  // We are node 0 (MY_ID). Neighbors: 1, 2, 3.
  routing_init();
  routing_set_link_state(1, LINK_HEALTHY);
  routing_set_link_state(2, LINK_HEALTHY);
  routing_set_link_state(3, LINK_HEALTHY);
  // Node 1 reaches node 3 in 1 hop (cost 1.0). Node 2 reaches node 4 in 1 hop (cost 1.0).
  RouteAdvEntry a1[] = { {3, 1.0f, 1}, {4, 2.0f, 2} };
  RouteAdvEntry a2[] = { {4, 1.0f, 1} };
  routing_on_adv(2, a2, 1);
  routing_on_adv(1, a1, 2);

  CHECK(routing_next_hop(3, false) == 3, "healthy: direct link to 3 is cheapest");
  CHECK(routing_next_hop(4, false) == 2, "node 4 reached via node 2 (cost 2.0)");
  CHECK_NEAR(routing_entry(4)->cost, 2.0f, "node 4 route cost is 2.00");

  routing_set_link_state(3, LINK_DEGRADING); // default penalty 5.0 -> direct cost 6.0
  CHECK(routing_next_hop(3, false) == 1, "3 DEGRADING: normal traffic reroutes via 1 (cost 2 < 6)");
  CHECK(routing_next_hop(3, true)  == 3, "3 DEGRADING: PRIORITY traffic still takes the 1-hop direct path");

  routing_set_link_state(3, LINK_DEAD);
  CHECK(routing_next_hop(3, false) == 1, "3 DEAD: normal via 1");
  CHECK(routing_next_hop(3, true)  == 1, "3 DEAD: priority also via 1 (dead links never used)");

  routing_set_link_state(1, LINK_DEAD);
  CHECK(routing_next_hop(3, false) == NO_ROUTE, "1 and 3 dead: 3 unreachable");
  CHECK(routing_next_hop(4, false) == 2, "node 4 still reachable via 2");

  // Poison reverse: we reach 4 via 2, so we must not advertise 4 back to node 2.
  RouteAdvEntry out[MAX_NODES];
  uint8_t k = routing_build_adv(2, out);
  bool leaked = false;
  for (uint8_t i = 0; i < k; i++) {
    if (out[i].dest == 4 && out[i].cost < ROUTE_INF) leaked = true;
  }
  CHECK(!leaked, "poison reverse: route to 4 not advertised back to 2");

  // ---- 3. Decimal Route Costs Distinction (e.g. 4.80 vs 5.30) ----
  routing_init();
  routing_set_link_state(1, LINK_DEGRADING, 3.80f); // cost = 1.0 + 3.8 = 4.80
  routing_set_link_state(2, LINK_DEGRADING, 4.30f); // cost = 1.0 + 4.3 = 5.30
  RouteAdvEntry adv1_dec[] = { {4, 0.0f, 1} };
  RouteAdvEntry adv2_dec[] = { {4, 0.0f, 1} };
  routing_on_adv(2, adv2_dec, 1); // path via 2: cost 5.30
  routing_on_adv(1, adv1_dec, 1); // path via 1: cost 4.80 (better by 0.50 > 0.10)
  CHECK(routing_next_hop(4, false) == 1, "decimal costs: router distinguishes 4.80 from 5.30");
  CHECK_NEAR(routing_entry(4)->cost, 4.80f, "decimal cost is 4.80");

  // ---- 4. Routing Hysteresis and Decimal Costs Tests ----
  routing_init();
  routing_set_link_state(1, LINK_HEALTHY); // linkCost[1] = 1.0f
  routing_set_link_state(2, LINK_HEALTHY); // linkCost[2] = 1.0f

  // Setup: Current route cost = 4.20 (via 1)
  RouteAdvEntry h1[] = { {4, 3.20f, 1} }; // 1.0 + 3.20 = 4.20
  routing_on_adv(1, h1, 1);
  CHECK(routing_next_hop(4, false) == 1, "setup: current route to 4 via 1 (cost 4.20)");
  CHECK_NEAR(routing_entry(4)->cost, 4.20f, "initial route cost is 4.20");

  // Current route cost = 4.20, Alternative = 4.35 -> current remains/4.20 selected
  RouteAdvEntry h2_worse[] = { {4, 3.35f, 1} }; // 1.0 + 3.35 = 4.35
  routing_on_adv(2, h2_worse, 1);
  CHECK(routing_next_hop(4, false) == 1, "alternative 4.35 > current 4.20: current remains");
  CHECK_NEAR(routing_entry(4)->cost, 4.20f, "cost remains 4.20");

#if USE_ROUTE_HYSTERESIS
  // Hysteresis: Current = 4.20, Alternative = 4.15 -> difference 0.05 <= 0.10 margin -> stay
  RouteAdvEntry h2_small[] = { {4, 3.15f, 1} }; // 1.0 + 3.15 = 4.15
  routing_on_adv(2, h2_small, 1);
  CHECK(routing_next_hop(4, false) == 1, "hysteresis: 4.20 vs 4.15 (diff 0.05 <= margin 0.10) -> stay on 1");
  CHECK_NEAR(routing_entry(4)->cost, 4.20f, "cost remains 4.20");

  // Hysteresis: Current = 4.20, Alternative = 3.90 -> difference 0.30 > 0.10 margin -> switch
  RouteAdvEntry h2_better[] = { {4, 2.90f, 1} }; // 1.0 + 2.90 = 3.90
  routing_on_adv(2, h2_better, 1);
  CHECK(routing_next_hop(4, false) == 2, "hysteresis: 4.20 vs 3.90 (diff 0.30 > margin 0.10) -> switch to 2");
  CHECK_NEAR(routing_entry(4)->cost, 3.90f, "cost switches to 3.90");

  // Dead current route: Node 2 dies -> switch immediately to Node 1 (no margin applied)
  routing_set_link_state(2, LINK_DEAD);
  CHECK(routing_next_hop(4, false) == 1, "dead current route: switch immediately to 1 (no margin applied)");
  CHECK_NEAR(routing_entry(4)->cost, 4.20f, "cost reverted to 4.20 via 1");
#else
  printf("(hysteresis disabled: ablation build, skipping hysteresis checks)\n");
#endif

  printf("ALL ROUTING TESTS PASSED\n");
  return 0;
}

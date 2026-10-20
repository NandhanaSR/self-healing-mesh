SRC = ../mesh_node
CXX = g++
FLAGS = -std=c++11 -Wall -Wextra -O1
all: sim_link_health test_routing test_anomaly
sim_link_health: sim_link_health.cpp $(SRC)/link_health.cpp
	$(CXX) $(FLAGS) -o $@ $^
test_routing: test_routing.cpp $(SRC)/routing.cpp $(SRC)/link_health.cpp
	$(CXX) $(FLAGS) -o $@ $^
test_anomaly: test_anomaly.cpp $(SRC)/anomaly.cpp
	$(CXX) $(FLAGS) -o $@ $^ -lm
ablation:
	@echo "== full (delta-PRR on) =="; $(CXX) $(FLAGS) -DUSE_DELTA_PRR=1 -o /tmp/sim_full sim_link_health.cpp $(SRC)/link_health.cpp && /tmp/sim_full | tail -3
	@echo "== w/o delta-PRR =="; $(CXX) $(FLAGS) -DUSE_DELTA_PRR=0 -o /tmp/sim_nodprr sim_link_health.cpp $(SRC)/link_health.cpp && /tmp/sim_nodprr | tail -3
	@echo "== interference scenario: full =="; /tmp/sim_full --interference | tail -2
	@echo "== interference scenario: w/o delta-PRR =="; /tmp/sim_nodprr --interference | tail -2
	@echo "== routing w/o hysteresis =="; $(CXX) $(FLAGS) -DUSE_ROUTE_HYSTERESIS=0 -o /tmp/rt_nohyst test_routing.cpp $(SRC)/routing.cpp $(SRC)/link_health.cpp && /tmp/rt_nohyst | tail -4
run: all
	./sim_link_health && ./test_routing && ./test_anomaly
clean:
	rm -f sim_link_health test_routing test_anomaly

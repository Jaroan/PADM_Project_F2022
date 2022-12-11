from pddl_parser.PDDL import PDDL_Parser
import sys, time

# '''
class Planner:

    def __init__(self):

        self.parser = PDDL_Parser()

    def solve(self, domain, problem):
        # Parser
        # parser = PDDL_Parser() 
        self.parser.parse_domain(domain)
        self.parser.parse_problem(problem)
        # Parsed data
        state = self.parser.state
        goal_pos = self.parser.positive_goals
        goal_not = self.parser.negative_goals
        # Do nothing
        if self.applicable(state, goal_pos, goal_not):
            return []
        # Grounding process
        ground_actions = []
        for action in self.parser.actions:
            for act in action.groundify(self.parser.objects, self.parser.types):
                ground_actions.append(act)
        # Search
        visited = set([state])
        fringe = [state, None]
        while fringe:
            state = fringe.pop(0)
            plan = fringe.pop(0)
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if new_state not in visited:
                        if self.applicable(new_state, goal_pos, goal_not):
                            full_plan = [act]
                            while plan:
                                act, plan = plan
                                full_plan.insert(0, act)

                            return full_plan
                        visited.add(new_state)
                        fringe.append(new_state)
                        fringe.append((act, plan))
        return None

    # -----------------------------------------------
    # Applicable
    # -----------------------------------------------

    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    # -----------------------------------------------
    # Apply
    # -----------------------------------------------

    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)


'''
# -----------------------------------------------
# Main
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()
    domain = sys.argv[1]
    problem = sys.argv[2]
    verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
    planner = Planner()
    plan = planner.solve(domain, problem)
    print('Time: ' + str(time.time() - start_time) + 's')
    if type(plan) is list:
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)

'''

# start_time = time.time()
# domain = 'domain.pddl'
# problem = 'problem.pddl'
# verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
# planner = Planner()
# plan = planner.solve(domain, problem)
# print('Time: ' + str(time.time() - start_time) + 's')
# if type(plan) is list:
#     print('plan:')
#     for act in plan:
#         # print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
#         print(act.name + ' ' + ' '.join(act.parameters))

# else:
#     print('No plan was found')
#     exit(1)
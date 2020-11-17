% max steps
#const n = 100.

% initial situation
holds(F,0) :- init(F).

% check goal situation
_ngoal(T) :- not holds(F,T), goal(F), T=0..n.
_ngoal(T) :- holds(F,T), ngoal(F), T=0..n.
:- _ngoal(n).

% cost
_has_cost(A) :- cost(A,C).
_cost(C,T) :- occurs(A,T), cost(A,C).
_cost(1,T) :- occurs(A,T), not _has_cost(A).
_costs(S) :- S = #sum{ C,T : _cost(C,T)}.

% optimization
#minimize {C : _costs(C)}.

%% state transition and state constraints
1 { occurs(A,_T) : _allowed(A,_T) } 1 :- _ngoal(_T-1), _T<=n.
holds(X,_T) :- holds(X,_T-1), not -holds(X,_T), _T<=n.

door(A,C,B) :- door(A,B,C).
locInRoom(A,B) :- door(A,B,C).
locInRoom(A,C) :- door(A,B,C).

% action close(G,D,R1,R2)
% state constraints
_allowed(close(G,D,R1,R2),_T) :- holds(empty(G),_T-1), door(D,R1,R2), holds(open(D),_T-1), holds(inRoom(robot,R1),_T-1), holds(isNear(robot,D),_T-1).
% state transition
holds(closed(D),_T) :- occurs(close(G,D,R1,R2),_T).
-holds(open(D),_T) :- occurs(close(G,D,R1,R2),_T).

% action pickup(O,L,G)
% state constraints
_allowed(pickup(O,L,G),_T) :- holds(empty(G),_T-1), graspable(O), holds(isNear(robot,L),_T-1), holds(isPlaced(O,L),_T-1).
% state transition
holds(isHeld(O,robot),_T) :- occurs(pickup(O,L,G),_T).
holds(inHand(O,G),_T) :- occurs(pickup(O,L,G),_T).
-holds(empty(G),_T) :- occurs(pickup(O,L,G),_T).
-holds(isPlaced(O,L),_T) :- occurs(pickup(O,L,G),_T).
% conditional state transition
-holds(inRoom(O,R),_T) :- occurs(pickup(O,L,G),_T), holds(inRoom(robot,R),_T-1).

% action putdown(O,L,G)
% state constraints
_allowed(putdown(O,L,G),_T) :- holds(inHand(O,G),_T-1), holds(isNear(robot,L),_T-1), isPlacement(L,true).
% state transition
holds(empty(G),_T) :- occurs(putdown(O,L,G),_T).
-holds(inHand(O,G),_T) :- occurs(putdown(O,L,G),_T).
-holds(isHeld(O,robot),_T) :- occurs(putdown(O,L,G),_T).
holds(isPlaced(O,L),_T) :- occurs(putdown(O,L,G),_T).
% conditional state transition
holds(inRoom(O,R),_T) :- occurs(putdown(O,L,G),_T), holds(inRoom(robot,R),_T-1).

% action moveIn(R2,R1,D)
% state constraints
_allowed(moveIn(R2,R1,D),_T) :- door(D,R1,R2), holds(open(D),_T-1), holds(inRoom(robot,R1),_T-1).
% state transition
holds(inRoom(robot,R2),_T) :- occurs(moveIn(R2,R1,D),_T).
-holds(inRoom(robot,R1),_T) :- occurs(moveIn(R2,R1,D),_T).
% conditional state transition
-holds(isNear(robot,L),_T) :- occurs(moveIn(R2,R1,D),_T), holds(isNear(robot,L),_T-1).

% action receive(O,H,G)
% state constraints
_allowed(receive(O,H,G),_T) :- holds(empty(G),_T-1), graspable(O), holds(isNear(robot,H),_T-1), holds(isHeld(O,H),_T-1).
% state transition
holds(isHeld(O,robot),_T) :- occurs(receive(O,H,G),_T).
holds(inHand(O,G),_T) :- occurs(receive(O,H,G),_T).
-holds(empty(G),_T) :- occurs(receive(O,H,G),_T).
-holds(isHeld(O,H),_T) :- occurs(receive(O,H,G),_T).

% action open(G,D,R1,R2)
% state constraints
_allowed(open(G,D,R1,R2),_T) :- holds(empty(G),_T-1), door(D,R1,R2), holds(inRoom(robot,R1),_T-1), holds(closed(D),_T-1), holds(isNear(robot,D),_T-1).
% state transition
holds(open(D),_T) :- occurs(open(G,D,R1,R2),_T).
-holds(closed(D),_T) :- occurs(open(G,D,R1,R2),_T).

% action moveTo(L,R)
% state constraints
_allowed(moveTo(L,R),_T) :- holds(inRoom(robot,R),_T-1), holds(inRoom(L,R),_T-1), not holds(isNear(robot,L),_T-1).
_allowed(moveTo(L,R),_T) :- holds(inRoom(robot,R),_T-1), locInRoom(L,R), not holds(isNear(robot,L),_T-1).
% state transition
holds(isNear(robot,L),_T) :- occurs(moveTo(L,R),_T).
% conditional state transition
-holds(isNear(robot,L1),_T) :- occurs(moveTo(L,R),_T), holds(isNear(robot,L1),_T-1).

% action findObj(O,L)
% state constraints
_allowed(findObj(O,L),_T) :- holds(isNear(robot,L),_T-1), object(O), holds(assume(isPlaced(O,L)),_T-1).
% state transition
holds(isPlaced(O,L),_T) :- occurs(findObj(O,L),_T).
holds(located(O,L),_T) :- occurs(findObj(O,L),_T).
-holds(assume(isPlaced(O,L)),_T) :- occurs(findObj(O,L),_T).
% conditional state transition
holds(inRoom(O,R),_T) :- occurs(findObj(O,L),_T), holds(inRoom(robot,R),_T-1).

% action give(O,H,G)
% state constraints
_allowed(give(O,H,G),_T) :- holds(isNear(robot,H),_T-1), holds(inHand(O,G),_T-1), human(H).
% state transition
holds(empty(G),_T) :- occurs(give(O,H,G),_T).
holds(isHeld(O,H),_T) :- occurs(give(O,H,G),_T).
-holds(inHand(O,G),_T) :- occurs(give(O,H,G),_T).
-holds(isHeld(O,robot),_T) :- occurs(give(O,H,G),_T).

% action accompany(H)
% state constraints
_allowed(accompany(H),_T) :- holds(isNear(robot,H),_T-1).
% state transition
holds(accompanied(H),_T) :- occurs(accompany(H),_T).

% action findPerson(H,L)
% state constraints
_allowed(findPerson(H,L),_T) :- human(H), holds(isNear(robot,L),_T-1), holds(assume(isNear(H,L)),_T-1).
% state transition
holds(isNear(H,L),_T) :- occurs(findPerson(H,L),_T).
holds(located(H,L),_T) :- occurs(findPerson(H,L),_T).
-holds(assume(isNear(H,L)),_T) :- occurs(findPerson(H,L),_T).
% conditional state transition
holds(inRoom(H,R),_T) :- occurs(findPerson(H,L),_T), holds(inRoom(robot,R),_T-1).

% show action sequence
#show occurs/2.
#show holds/2.
#show -holds/2.
#show goal/1.

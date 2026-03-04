package frc.robot;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

// Learning how FSM works using WPILib's Command Base before switching to a 6328 implementation.
/*
 * Gist:
 * public BobaDispenser createStateMachine() {
 * var stateMachine = new BobaDispenser("Boba FSM");
 * Command cmd1 = Commands.runOnce(()-> );
 * Command cmd2 = Commands.runOnce(()-> );
 * * State state1 = stateMachine.addState("State1", cmd1);
 * State state2 = stateMachine.addState("State2", cmd2);
 * * stateMachine.setInitialState(state1);
 * * BooleanSupplier condition1 = () ->  foo;
 * * //Without mapping
 * state1.switchTo(state2).whenComplete();
 * state2.switchTo(state1).when(condition1);
 * * return stateMachine;
 * }
 * * createStateMachine().schedule(); //CommandScheduler handles queuing and running each state.
 */

// You only need to initialize the FSM once, then you can schedule it whenever you want.
/*
 * Command-Based FSM: This BobaDispenser class is itself a WPILib. Ideally, you can
 * schedule the entire machine to run and it would handle each of the subsystem requirements
 * used in the command for each state. Where it differs from a normal WPILib Command is that
 * instead of single-looping the execute() method, the FSM dynamically schedules and cancels
 *  other internal commands (the States) based on the transitions and rules you set up.
 */
/*
 * Flow Map:
 * 1. Every 20ms, BobaDispenser.execute() polls the event loop.
 * 2. If a condition defined in the current state's transitions = true, the corresponding
 *    trigger event is enabled.
 * 3. The trigger's action is to schedule the next state's command accessed by
 *    transition.nextState.stateCommandAugmented. This command is a WrapState that wraps
 *    the original state command with code to handle the transition and trigger management.
 * 4. Scheduling a new command automatically interrupts any currently running command that
 *    requires the same subsystem. Thus, call end() for the old state to clear TRIGGERS.
 * 5. The new state's initialize() method runs, setting up its state-specific TRIGGERS.
 * 6. Repeat the cycle.
 */

// Note: Don't make random things null. If you need a "default" state, make an actual
// state for it and set it as the initial state.
public class BobaDispenser extends Command {

  private String name = "not chosen"; // name of the FSM
  private boolean exitStateMachine = false; // marks whether the FSM is to exit (end)
  private EventLoop events = new EventLoop(); // polled for TRIGGERS
  private final List<State> states = new ArrayList<>(); // instantiated STATES
  private State initialState = null; // MUST call setInitialState before scheduling the FSM
  private State completedNormally = null; // marks for whenComplete() TRIGGER
  private Command stateCommandAugmentedPrevious =
      null; // need to know if previous is still running so can be cancelled on state transition
  private int countSimultaneousTransitions =
      0; // check for multiple simultaneous transition triggers

  public BobaDispenser(String name) {
    requireNonNullParam(name, "name", "BobaDispenser");
    this.name = name;
  }

  // When this StateMachine is scheduled by CommandScheduler, it will run the initialState's command
  // and poll for triggers to transition to other states.
  public void setInitialState(State initialState) {
    requireNonNullParam(initialState, "initialState", "StateMachine.setInitialState");
    this.initialState = initialState;
  }

  // Connect(s) command to a semantic keyword (state name) for easier readability and organization.
  public State addState(String name, Command stateCommand) {
    return new State(name, stateCommand);
  }

  // Now set-up transitions from any current state to any other requested state.
  /*
   * stateMachine.switchFromAny(state1, state2, state3).to(state4).when(() -> condition);
   * Functionally equivalent to:
   * state1.switchTo(state4).when(() -> condition);
   * state2.switchTo(state4).when(() -> condition);
   * state3.switchTo(state4).when(() -> condition);
   */
  public TransitionNeedsTargetStage switchFromAny(State... states) {
    if (states.length == 0) {
      return new TransitionNeedsTargetStage(List.copyOf(this.states));
    } else {
      return this.new TransitionNeedsTargetStage(List.of(states));
    }
  }

  // Establish the handler for transitioning from the "from" state(s) to the "to" state.
  /*
   * 1. Selecting a Command: Choose any amount of originStates and This returns a
   *    TransitionNeedsTargetStage builder.
   * 2. Select Target State: Choose a target state to transition to. This passes the origin
   *    and destination into the next builder: TransitionNeedsConditionStage
   * 3. Check for TRIGGERS: .when() method takes a BooleanSupplier and creates a
   *    Transition object stored inside the origin state
   */
  /*
  * Code Example: stateA.switchTo(stateB).when(condition);
  * 1. stateA.switchTo(stateB)
  *    Method is defined inside the State inner class
  * What is Happening: You are telling stateA that it is the Origin.
  *    Then, calls .to(to) on a temporary builder.
  * Final Result: It returns an instance of TransitionNeedsConditionStage.
  * However, the object only holds stateA as the source and stateB as the
  * target, but nothing has been saved to the FSM yet because it lacks a trigger.
  * 2. TransitionNeedsConditionStage
  *public final class TransitionNeedsConditionStage {
       private final List<State> m_originatingStates; // Holds stateA
       private final State m_targetState;             // Holds stateB

   // Constructor simply stores the references
   private TransitionNeedsConditionStage(List<State> from, State to) {
       m_originatingStates = from;
       m_targetState = to;
   }
  *3. .when(condition) actually modifies the State object
  *
  */
  public final class TransitionNeedsTargetStage {
    private final List<State> m_from;

    private TransitionNeedsTargetStage(List<State> from) {
      m_from = from;
    }

    // Designates the "to" state for the transition and returns the next stage of the transition
    // builder.
    public TransitionNeedsConditionStage to(State to) {
      return new TransitionNeedsConditionStage(m_from, to);
    }

    // Special case for when transition transition to the "exit" state (end the FSM) instead of
    // another state
    public TransitionNeedsConditionStage toExitStateMachine() {
      return new TransitionNeedsConditionStage(m_from, null);
    }
  }

  // ----------------

  // Builder that set conditions for a transition from one state to another
  // Use when(BooleanSupplier condition) for conditional transitions, and whenComplete() for
  // completion transitions.
  public final class TransitionNeedsConditionStage {
    private final List<State> m_originatingStates;
    private final State m_targetState; // If null, it commits seppuku on scheduled.

    private TransitionNeedsConditionStage(List<State> from, State to) {
      m_originatingStates = from;
      m_targetState = to;
    }

    // Builds a transition that will be triggered when "condition" is true
    public void when(BooleanSupplier condition) {
      // Create the actual data object representing the link
      var transition = new Transition(m_targetState, condition);
      // Loop through origin states (stateA)
      m_originatingStates.forEach(
          originatingState -> {
            checkDuplicateCondition(originatingState, condition);
            // strictly adds the transition to stateA's internal list
            originatingState.transitions.add(
                transition); // wrap condition and add to the list a transition to this state
          });
    }

    // Marks when a state is completed but hasn't had any other conditions for transitions yet.
    public void whenComplete() {
      m_originatingStates.forEach(
          originatingState -> {
            checkDuplicateCondition(originatingState, originatingState.whenCompleteCondition);
            var transition = new Transition(m_targetState, originatingState.whenCompleteCondition);
            originatingState.transitions.add(
                transition); // wrap condition and add to the list a transition to this state
          });
    }

    private void checkDuplicateCondition(State originatingState, BooleanSupplier condition) {
      for (Transition transition : originatingState.transitions) {
        if (transition.triggeringEvent == condition) {
          throw new IllegalArgumentException("ONLY USE ONE CONDITION PER STATE.");
        }
      }
    }
  }

  // -----------------
  // Goober Debugging Code
  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();

    sb.append("All states for StateMachine " + name + "\n");

    for (State state : states) {
      boolean noExits = true; // initially haven't found any
      boolean noEntrances = true; // initially haven't found any

      sb.append("------- " + state.name + " -------\n");
      sb.append(state == initialState ? "INITIAL STATE\n" : "");
      // loop through all the transitions of this state
      for (Transition transition : state.transitions) {
        noExits = false; // at least one transition out of this state
        sb.append(
            "transition "
                + transition
                + " to "
                + (transition.nextState != null ? transition.nextState.name : "exit StateMachine")
                + " onTrue trigger "
                + transition.triggeringEvent
                + "\n");
      }

      // loop through all the states again to find at least one entrance to this state
      allStates:
      for (State stateInner : states) {
        for (Transition transition : stateInner.transitions) {
          if (transition.nextState == state) {
            noEntrances = false;
            break allStates;
          }
        }
      }
      sb.append(
          (noEntrances && state != initialState
              ? "Caution - State has no entrances and will not be used.\n\n"
              : noExits
                  ? "Notice - State has no exits and if entered will either stop or hang the StateMachine command.\n\n"
                  : "\n"));
    }
    return sb.toString();
  }

  // ---------------
  // Command Structure for the FSM

  // Called on Init
  @Override
  public void initialize() {
    exitStateMachine = false;
    initialState.stateCommandAugmented.schedule();
  }

  // Called repeatedly (not necessarily every 20ms) while the FSM is running to check for TRIGGERs
  /*
   * Create a private instance of the CommandScheduler Event Loop to check for the TRIGGERs that
   * causes transitions. This was chosen so that polling only occurs when the FSM is actually
   * running. If commands are canceled or finishes, all internal state transitions are disabled.
   */
  @Override
  public void execute() {
    if (countSimultaneousTransitions > 1) {
      DriverStation.reportWarning("Multiple states triggered simultaneously", false);
    }
    countSimultaneousTransitions = 0;
    events.poll(); // check for events that trigger transitions
  }

  // Call to decapitate the FSM (stop it from running) either by a transition to the "exit" state or
  // by canceling the command.
  @Override
  public void end(boolean interrupted) {
    // cancel the State command if it's still running
    if (stateCommandAugmentedPrevious != null) {
      stateCommandAugmentedPrevious.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return exitStateMachine; // boolean check if last state command ordered StateMachine to stop
  }

  // -----------------

  // Wrapper Command to envelope a state command with code to handle transitions and TRIGGERs
  /*
   * Most of the magic stuff happens here. When you add a state using addState(), it creates a
   * WrapState command. This wrapper allows the FSM to choose when to run the state command.
   * When a state is scheduled to run, the wrapper's initialize() method clears the FSM's custom
   * event loop and registers only the TRIGGERS need for this new state. This cuts down on lag.
   * When the state ends, the TRIGGERS are discarded so that they don't interfere with the next
   * state.
   */
  private class WrapState extends WrapperCommand {
    State state;

    WrapState(State state, Command command) {
      super(command); // original state command to run
      this.state = state;
    }

    @Override
    public void initialize() {
      events.clear(); // wipe the previous state's triggers
      if (stateCommandAugmentedPrevious != null) {
        stateCommandAugmentedPrevious
            .cancel(); // wipe the previous state in case it didn't finish itself
      }
      // make triggers for all of this state's transitions
      // if no transitions, that will be handled later as an exit but first need to run this state
      if (!state.transitions.isEmpty()) {
        for (Transition transition : state.transitions) { // add all the events for this state
          var trigger =
              new Trigger(
                  events, transition.triggeringEvent); // for .when(condition) and .whenComplete()
          trigger.onTrue(
              Commands.runOnce(() -> ++countSimultaneousTransitions)
                  .ignoringDisable(true)); // for check erroneous multiple identical conditions
          if (transition.nextState == null) { // condition for .exitStateMachine()
            trigger.onTrue(
                Commands.runOnce(() -> exitStateMachine = true)
                    .ignoringDisable(true)); // mark to exit (end) FSM
          } else { // condition to trigger next state
            trigger.onTrue(transition.nextState.stateCommandAugmented); // start next state
          }
        }
      }

      completedNormally =
          null; // reset flag for this new state as it has not yet completed normally 'cuz it's just
      // starting
      stateCommandAugmentedPrevious = this; // for next state change this will be the previous state

      m_command
          .initialize(); // Wrapper is done with its fussing so tell original command to initialize
    }

    @Override
    public void end(boolean interrupted) {
      m_command.end(interrupted); // tell original command to end and if interrupted or not

      // setup for the next state or exit
      stateCommandAugmentedPrevious =
          null; // indicate state already ended so there is not a previous state to cancel

      if (state.transitions.isEmpty()) { // no transitions [no .when() nor .whenComplete()]
        exitStateMachine =
            true; // no matter how this state ended tell StateMachine to exit since nowhere to go
        // from here
      } else {
        if (!interrupted) {
          completedNormally = state; // indicate state ended by itself without others help
          // see if this state has transition .exitStateMachine().whenComplete()
          for (Transition transition : state.transitions) { // check all transitions
            if (transition.triggeringEvent == state.whenCompleteCondition) { // for .whenComplete()
              if (transition.nextState == null) { // for .exitStateMachine()
                exitStateMachine = true;
              }
              break; // don't look for any more since cannot be more than one whenComplete trigger
            }
          }
        }
      }
    }
  } // end WrapState

  // -----------------
  public class State extends Command {
    private final String name;
    private Command
        stateCommandAugmented; // the Wrapped (instrumented) state command that will actually be run
    private List<Transition> transitions =
        new ArrayList<Transition>(); // the transitions for this State
    private BooleanSupplier whenCompleteCondition =
        () -> State.this == completedNormally; // trigger condition for whenComplete

    private State(String name, Command stateCommand) {
      this.name = name;
      BobaDispenser.this.states.add(this);
      this.stateCommandAugmented = new WrapState(this, stateCommand);
    }

    public TransitionNeedsConditionStage switchTo(State to) {
      requireNonNullParam(to, "to", "State.switchTo");
      return new TransitionNeedsTargetStage(List.of(this)).to(to);
    }

    public TransitionNeedsConditionStage exitStateMachine() {
      return new TransitionNeedsConditionStage(List.of(this), null);
    }
  }

  // Define the FSM transition as current state + triggering event -> next state
  private class Transition {
    State nextState;
    BooleanSupplier triggeringEvent;

    private Transition(State toNextState, BooleanSupplier whenEvent) {
      this.nextState = toNextState;
      this.triggeringEvent = whenEvent;
    }
  }
}

#include "homing.h"
#include "config.h"
#include "limits.h"
#include "axis.h"

void startHoming(Axis& ax)
{
  if (!ax.enabled) setEnable(ax, true);

  ax.homed = false;
  ax.maxPos = 0;

  ax.stepper.setAcceleration(HOME_ACCEL);
  ax.stepper.setMaxSpeed(HOME_FAST_SPEED);

  // Start by seeking lower limit (negative direction assumed)
  ax.stepper.moveTo(-BIG_TRAVEL);
  ax.hs = HomeState::SEEK_LOWER_FAST;
}

void updateHoming(Axis& ax)
{
  switch (ax.hs)
  {
    case HomeState::IDLE:
    case HomeState::DONE:
    case HomeState::ERROR:
      return;

    // -------- Lower limit --------
    case HomeState::SEEK_LOWER_FAST: 
    {
      ax.stepper.run();
      if (limitTriggered(ax.limLoPin))
      {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::BACKOFF_FROM_LOWER;
      }
      return;
    }

    case HomeState::BACKOFF_FROM_LOWER: 
    {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.move(BACKOFF_STEPS);
      ax.hs = HomeState::SEEK_LOWER_SLOW;
      return;
    }

    case HomeState::SEEK_LOWER_SLOW: 
    {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0)
      {
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.moveTo(-BIG_TRAVEL);
      }
      if (limitTriggered(ax.limLoPin))
      {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::CLEAR_LOWER_FINAL;
      }
      return;
    }

    case HomeState::CLEAR_LOWER_FINAL: 
    {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
      ax.stepper.move(FINAL_CLEAR_STEPS);
      ax.hs = HomeState::SET_ZERO;
      return;
    }

    case HomeState::SET_ZERO: 
    {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() != 0) return;

      ax.stepper.setCurrentPosition(0);

      // -------- Upper limit --------
      ax.stepper.setAcceleration(HOME_ACCEL);
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.moveTo(+BIG_TRAVEL);
      ax.hs = HomeState::SEEK_UPPER_FAST;
      return;
    }

    // -------- Upper limit --------
    case HomeState::SEEK_UPPER_FAST: 
    {
      ax.stepper.run();
      if (limitTriggered(ax.limHiPin))
      { 
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::BACKOFF_FROM_UPPER;
      }
      return;
    }

    case HomeState::BACKOFF_FROM_UPPER: 
    {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.move(-BACKOFF_STEPS);
      ax.hs = HomeState::SEEK_UPPER_SLOW;
      return;
    }

    case HomeState::SEEK_UPPER_SLOW: 
    {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0)
      {
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.moveTo(+BIG_TRAVEL);
      }
      if (limitTriggered(ax.limHiPin))
      {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::CLEAR_UPPER_FINAL;
      }
      return;
    }

    case HomeState::CLEAR_UPPER_FINAL: 
    {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
      ax.stepper.move(-FINAL_CLEAR_STEPS);
      ax.hs = HomeState::SET_MAX;
      return;
    }

    case HomeState::SET_MAX: 
    {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() != 0) return;

      // max position corresponds to the upper edge; since we cleared away FINAL_CLEAR_STEPS,
      // approximate upper edge as current + FINAL_CLEAR_STEPS.
      ax.maxPos = ax.stepper.currentPosition() + FINAL_CLEAR_STEPS;

      long mid = ax.maxPos / 2;
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.moveTo(mid);
      ax.hs = HomeState::MOVE_TO_MID;
      return;
    }

    case HomeState::MOVE_TO_MID: 
    {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0)
      { 
        ax.homed = true;
        ax.hs = HomeState::DONE;
      }
      return;
    }

    default:
      ax.hs = HomeState::ERROR;
      return;
  }
}
#include "homing.h"
#include "config.h"
#include "limits.h"
#include "axis.h"

void startHoming(Axis& ax)
{
  if (!ax.enabled) setEnable(ax, true);

  ax.homed = false;
  ax.posValid = false;
  ax.maxPos = 0;
  ax.homeStartMs = millis();

  ax.stepper.setAcceleration(HOME_ACCEL);
  ax.stepper.setMaxSpeed(HOME_FAST_SPEED);

  // Start by seeking lower limit (negative direction assumed)
  ax.stepper.moveTo(-BIG_TRAVEL);
  ax.hs = HomeState::SEEK_LOWER_FAST;
}

void updateHoming(Axis& ax)
{
  // Global homing timeout: abort if homing takes too long without reaching completion.
  if (ax.hs != HomeState::IDLE &&
      ax.hs != HomeState::DONE &&
      ax.hs != HomeState::ERROR)
  {
    if ((millis() - ax.homeStartMs) >= HOME_TIMEOUT_MS)
    {
      ax.stepper.stop();
      ax.stepper.run();   // allow stop command to begin taking effect
      setEnable(ax, false);
      ax.hs = HomeState::ERROR;
      ax.homed = false;
      ax.posValid = false;
      return;
    }
  }

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

        ax.hs = HomeState::BACKOFF_FROM_LOWER;
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.move(BACKOFF_STEPS);
      }
      return;
    }

    case HomeState::BACKOFF_FROM_LOWER:
    {
      ax.stepper.run();
      if (!(ax.stepper.distanceToGo() == 0)) return;

      ax.hs = HomeState::SEEK_LOWER_SLOW;
      ax.stepper.move(-BIG_TRAVEL);
      return;
    }

    case HomeState::SEEK_LOWER_SLOW:
    {
      ax.stepper.run();
      if (limitTriggered(ax.limLoPin))
      {
        ax.stepper.stop();

        ax.hs = HomeState::CLEAR_LOWER_FINAL;
        ax.stepper.move(FINAL_CLEAR_STEPS);
      }
      return;
    }

    case HomeState::CLEAR_LOWER_FINAL:
    {
      ax.stepper.run();
      if (!(ax.stepper.distanceToGo() == 0))
      {
        ax.stepper.stop();
        ax.t_ms = millis();
        return;
      } 
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.stepper.setCurrentPosition(0);

      // -------- Upper limit --------
      ax.hs = HomeState::SEEK_UPPER_FAST;
      ax.stepper.setAcceleration(HOME_ACCEL);
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.moveTo(+BIG_TRAVEL);
      delay(1000);

      return;
    }

    // -------- Upper limit --------
    case HomeState::SEEK_UPPER_FAST:
    {
      ax.stepper.run();
      if (limitTriggered(ax.limHiPin))
      {
        ax.stepper.stop();
        
        ax.hs = HomeState::BACKOFF_FROM_UPPER;
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.move(-BACKOFF_STEPS);
      }
      return;
    }

    case HomeState::BACKOFF_FROM_UPPER:
    {
      ax.stepper.run();
      if (!(ax.stepper.distanceToGo() == 0)) return;

      ax.hs = HomeState::SEEK_UPPER_SLOW;
      ax.stepper.move(+BIG_TRAVEL);
      return;
    }

    case HomeState::SEEK_UPPER_SLOW:
    {
      ax.stepper.run();
      if (limitTriggered(ax.limHiPin))
      {
        ax.stepper.stop();
        ax.t_ms = millis();

        ax.hs = HomeState::CLEAR_UPPER_FINAL;
        ax.stepper.move(-FINAL_CLEAR_STEPS);
      }
      return;
    }

    case HomeState::CLEAR_UPPER_FINAL:
    {
      ax.stepper.run();
      if (!(ax.stepper.distanceToGo() == 0))
      {
        ax.stepper.stop();
        ax.t_ms = millis();
        return;
      } 
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      ax.maxPos = ax.stepper.currentPosition();

      ax.hs = HomeState::DONE;
      ax.homed = true;
      ax.posValid = true;
      setEnable(ax, false);   // disable motor after homing is complete
      return;
    }

    default:
      ax.stepper.stop();
      setEnable(ax, false);
      ax.homed = false;
      ax.posValid = false;
      ax.hs = HomeState::ERROR;
      return;
  }
}
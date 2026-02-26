def main() -> None:
    try:
        from .sim import FlightSimulatorApp
    except ModuleNotFoundError as exc:
        if exc.name == "pygame":
            print("Missing dependency: pygame")
            print("Install dependencies first, then run with your project venv:")
            print("  pip install -r requirements.txt")
            print("  ./.venv/bin/python -m flightsim")
            raise SystemExit(1)
        raise

    app = FlightSimulatorApp()
    app.run()


if __name__ == "__main__":
    main()

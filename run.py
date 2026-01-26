import subprocess
import sys

def run_mpremote(args):
    """Run mpremote with the given arguments using uv."""
    cmd = ["uv", "run", "mpremote"] + args
    print(f"Running: {' '.join(cmd)}")
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        print(f"Error: Command failed with exit code {e.returncode}")
        sys.exit(e.returncode)

def main():
    # Commands to run: reset then repl
    # Using + to separate commands in mpremote
    args = ["reset", "+", "repl"]
    
    print("Restarting and connecting to REPL...")
    run_mpremote(args)

if __name__ == "__main__":
    main()

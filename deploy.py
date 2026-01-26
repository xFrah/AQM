import subprocess
import sys
import os

def run_mpremote(args):
    """Run mpremote with the given arguments using uv."""
    cmd = ["uv", "run", "mpremote"] + args
    print(f"Running: {' '.join(cmd)}")
    try:
        # Use simple subprocess call. 
        # For 'repl', it connects to stdio, so we don't capture output.
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        print(f"Error: Command failed with exit code {e.returncode}")
        sys.exit(e.returncode)

def main():
    commands = []
    
    # 1. Copy lib folder to /lib
    if os.path.exists("lib"):
        print("Queueing lib directory copy...")
        # 'fs cp -r lib :' copies the local 'lib' directory to remote ':' (root)
        commands.append(["fs", "cp", "-r", "lib", ":"])

    # 2. Copy src contents to root
    if os.path.exists("src"):
        print("Queueing src files copy...")
        for root, dirs, files in os.walk("src"):
            for file in files:
                local_path = os.path.join(root, file)
                # Calculate relative path from src to put at root
                rel_path = os.path.relpath(local_path, "src")
                # Ensure forward slashes for remote path
                remote_path = ":" + rel_path.replace(os.sep, "/")
                
                commands.append(["fs", "cp", local_path, remote_path])

    # 3. Reset and enter REPL
    commands.append(["reset"])
    commands.append(["repl"])

    # Flatten arguments with "+" separator
    final_args = []
    for i, cmd in enumerate(commands):
        final_args.extend(cmd)
        if i < len(commands) - 1:
            final_args.append("+")

    print("Executing deployment...")
    run_mpremote(final_args)

if __name__ == "__main__":
    main()

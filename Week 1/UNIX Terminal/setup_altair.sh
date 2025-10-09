TARGET_DIR="$HOME/altair_software_recruitment/executables"
PYTHON_FILE="$TARGET_DIR/hello_altair.py"
mkdir -p "$TARGET_DIR"

cat <<EOF > "$PYTHON_FILE"
#!/usr/bin/env python3
print("Hello Altair!")
EOF

sudo chown root:root "$PYTHON_FILE"
sudo chmod 700 "$PYTHON_FILE"
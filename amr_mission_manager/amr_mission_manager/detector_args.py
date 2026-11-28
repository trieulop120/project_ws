"""ROS launch arguments shared by PICK prewarm and Detect ON."""


def with_cargo_classes(args, classes):
    # Replace a prior filter pair if present; leave all other launch args.
    clean, i = [], 0
    while i < len(args):
        arg = args[i]
        if arg == '-p' and i + 1 < len(args) and args[i + 1].startswith('target_cargo_classes:='):
            i += 2
            continue
        if arg.startswith('target_cargo_classes:='):
            i += 1
            continue
        clean.append(arg)
        i += 1
    clean.extend(['-p', 'target_cargo_classes:=' + ','.join(classes)])
    return clean

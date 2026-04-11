#!/usr/bin/env python3
"""Launch the Scanner Debug Mode web interface."""

import argparse
import uvicorn


def main():
    parser = argparse.ArgumentParser(description='Scanner Debug Mode')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8000,
                        help='Port to listen on')
    parser.add_argument('--reload', action='store_true',
                        help='Auto-reload on code changes')
    args = parser.parse_args()

    uvicorn.run(
        'web.debug_app:app',
        host=args.host,
        port=args.port,
        reload=args.reload,
    )


if __name__ == '__main__':
    main()

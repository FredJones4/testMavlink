import logging
import asyncio
from mavsdk import System

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    logging.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            logging.info("Drone connected!")
            break

    # Your other code here

if __name__ == "__main__":
    asyncio.run(run())

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Config.hpp"

#include "AStar.hpp"
#include "BoundedVector.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "ModelObject.hpp"
#include "Observer.hpp"
#include "Point.hpp"
#include "Region.hpp"
#include "Size.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#define ROBOT_WARNING_DISTANCE 100
#define ROBOT_RESTART_DISTANCE 115

namespace Messaging
{
	class Message;
	class Server;
	typedef std::shared_ptr< Server > ServerPtr;
}

namespace Model
{
	class Robot;
	typedef std::shared_ptr< Robot > RobotPtr;

	class Goal;
	typedef std::shared_ptr< Goal > GoalPtr;

	/**
	 *
	 */
	class Robot :	public ModelObject,
					public Messaging::MessageHandler,
					public Base::Observer
	{
		public:
			/**
			 *
			 */
			Robot();
			/**
			 *
			 */
			explicit Robot( const std::string& aName);
			/**
			 *
			 */
			Robot(	const std::string& aName,
					const wxPoint& aPosition);
			/**
			 *
			 */
			virtual ~Robot();
			/**
			 *
			 */
			std::string getName() const
			{
				return name;
			}
			/**
			 *
			 */
			void setName(	const std::string& aName,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			wxSize getSize() const;
			/**
			 *
			 */
			void setSize(	const wxSize& aSize,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			wxPoint getPosition() const
			{
				return position;
			}
			/**
			 *
			 */
			void setPosition(	const wxPoint& aPosition,
								bool aNotifyObservers = true);
			/**
			 *
			 */
			BoundedVector getFront() const;
			/**
			 *
			 */
			void setFront(	const BoundedVector& aVector,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			float getSpeed() const;
			/**
			 *
			 */
			void setSpeed( 	float aNewSpeed,
							bool aNotifyObservers = true);
			/**
			 *
			 * @return true if the robot is acting, i.e. either planning or driving
			 */
			bool isActing() const
			{
				return acting;
			}
			/**
			 *
			 */
			virtual void startActing();
			/**
			 *
			 */
			virtual void stopActing();
			/**
			 *
			 * @return true if the robot is driving
			 */
			bool isDriving() const
			{
				return driving;
			}
			/**
			 *
			 */
			virtual void startDriving();
			/**
			 *
			 */
			virtual void stopDriving();
			/**
			 *
			 * @return true if the robot is communicating, i.e. listens with an active ServerConnection
			 */
			bool isCommunicating() const
			{
				return communicating;
			}
			/**
			 * Starts a ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 */
			void startCommunicating();
			/**
			 * Connects to the ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 * and sends a message with messageType "1" and a body with "stop"
			 *
			 */
			void stopCommunicating();
			/**
			 *  Sends the locations of the goals, walls and robots in a world to another world
			 */
			void sendWorldInfo();
			/**
			 *
			 */
			wxRegion getRegion() const;
			/**
			 *
			 */
			bool intersects( const wxRegion& aRegion) const;
			/**
			 * Returns true if the distance between the midpoint of this robot and another robot is smaller than a certain maximum; false if not
			 */
			bool closeToOtherRobot(double maximumDistance) const;
			/**
			 * Clears the previous path created by AStar
			 */
			void clearPath();
			/**
			 *
			 */
			wxPoint getFrontLeft() const;
			/**
			 *
			 */
			wxPoint getFrontRight() const;
			/**
			 *
			 */
			wxPoint getBackLeft() const;
			/**
			 *
			 */
			wxPoint getBackRight() const;
			/**
			 * @name Observer functions
			 */
			//@{
			/**
			 * A Notifier will call this function if this Observer will handle the notifications of that
			 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
			 *
			 */
			virtual void handleNotification() override;
			//@}
			/**
			 *
			 */
			PathAlgorithm::OpenSet getOpenSet() const
			{
				return astar.getOpenSet();
			}
			/**
			 *
			 */
			PathAlgorithm::Path getPath() const
			{
				return path;
			}
			/**
			 * @name Messaging::MessageHandler functions
			 */
			//@{
			/**
			 * This function is called by a ServerSesssion whenever a message is received. If the request is handled,
			 * any response *must* be set in the Message argument. The message argument is then echoed back to the
			 * requester, probably a ClientSession.
			 *
			 * @see Messaging::RequestHandler::handleRequest( Messaging::Message& aMessage)
			 */
			virtual void handleRequest( Messaging::Message& aMessage);
			/**
			 * This function is called by a ClientSession whenever a response to a previous request is received.
			 *
			 * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
			 */
			virtual void handleResponse( const Messaging::Message& aMessage);
			//@}
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}

			/**
			 * @name Variables for painting the sensor activity on the screen
			 */
			//@{
			// The start point of this run
			wxPoint startPosition;
			//@}
			/**
			 * Extracts the numbers from a string if they are separated by commas
			 * @param stringWithNumbers The string to extract the numbers from
			 * @return A vector of numbers extracted from the string
			 */
			std::vector<int> getNumbersFromString(std::string stringWithNumbers);
			/**
			 * Puts the goal(s) from another robotworld in the current robotworld
			 * @param goalString The position(s) of the goal(s) from another world
			 */
			void syncGoals(std::string& goalString);
			/**
			 * Puts the wall(s) from another robotworld in the current robotworld
			 * @param wallString The position(s) of the wall(s) from another world
			 */
			void syncWalls(std::string& wallString);
			/**
			 * Puts the positions of the goals, walls, and robots of the current world into a string
			 * @return The string with the information of the world
			 */
			std::string getWorldInfo();
			/**
			 * Syncs the robot after a request
			 * @param robotString The position and rotation of the robot
			 */
			void syncRobot(std::string& robotString);
		protected:
			/**
			 * Adds a robot based on the info from a client server message
			 * @param robotString A string that contains the info of the robot
			 */
			void addNewRobot(std::string& robotString);
			/**
			 * Sends the position of the robot over and over
			 */
			void sendRobotPosMessage();
			/**
			 *
			 */
			void drive();
			/**
			 *
			 */
			void calculateRoute( GoalPtr aGoal);
			/**
			 *
			 */
			bool arrived( GoalPtr aGoal);
			/**
			 *
			 */
			bool collision();
			/**
			*
			 */

		private:
			/**
			 *
			 */
			std::string name;
			/**
			 *
			 */
			wxSize size;
			/**
			 *
			 */
			wxPoint position;
			/**
			 *
			 */
			BoundedVector front;
			/**
			 *
			 */
			float speed;
			/**
			 *
			 */
			GoalPtr goal;
			/**
			 *
			 */
			PathAlgorithm::AStar astar;
			/**
			 *
			 */
			PathAlgorithm::Path path;
			/**
			 *
			 */
			bool acting;
			/**
			 *
			 */
			bool driving;
			/**
			 *
			 */
			bool communicating;
			/**
			 * True if the robot is within a certain distance (determined by ROBOT_WARNING_DISTANCE) of another robot, false if not.
			 */
			bool almostCollided;
			/**
			 * True if 'Sync world' has been pressed in the application before driving, false if not.
			 */
			bool worldSyncer;
			/**
			 *
			 */
			std::thread robotThread;
			/**
			 *
			 */
			mutable std::recursive_mutex robotMutex;
			/**
			 *
			 */
			Messaging::ServerPtr server;
	};
} // namespace Model
#endif // ROBOT_HPP_

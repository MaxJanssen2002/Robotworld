#include "Robot.hpp"

#include "Client.hpp"
#include "CommunicationService.hpp"
#include "Goal.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MathUtils.hpp"
#include "Message.hpp"
#include "MessageTypes.hpp"
#include "RobotWorld.hpp"
#include "Server.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "WayPoint.hpp"

#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <regex>

namespace Model
{
	/**
	 *
	 */
	Robot::Robot() : Robot("", wxDefaultPosition)
	{
	}
	/**
	 *
	 */
	Robot::Robot( const std::string& aName) : Robot(aName, wxDefaultPosition)
	{
	}
	/**
	 *
	 */
	Robot::Robot(	const std::string& aName,
					const wxPoint& aPosition) :
								name( aName),
								size( wxDefaultSize),
								position( aPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false),
								almostCollided(false),
								worldSyncer(false)
	{
		// We use the real position for starters, not an estimated position.
		startPosition = position;
		if(Application::MainApplication::isArgGiven("-name"))
		{
			name = Application::MainApplication::getArg("-name").value;
		}
	}
	/**
	 *
	 */
	Robot::~Robot()
	{
		Application::Logger::log(__PRETTY_FUNCTION__);
		if(driving)
		{
			Robot::stopDriving();
		}
		if(acting)
		{
			Robot::stopActing();
		}
		if(communicating)
		{
			stopCommunicating();
		}
	}
	/**
	 *
	 */
	void Robot::setName( const std::string& aName,
						 bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	wxSize Robot::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void Robot::setSize(	const wxSize& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::setPosition(	const wxPoint& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	BoundedVector Robot::getFront() const
	{
		return front;
	}
	/**
	 *
	 */
	void Robot::setFront(	const BoundedVector& aVector,
							bool aNotifyObservers /*= true*/)
	{
		front = aVector;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	float Robot::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void Robot::setSpeed( float aNewSpeed,
						  bool aNotifyObservers /*= true*/)
	{
		speed = aNewSpeed;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::startActing()
	{
		acting = true;
		std::thread newRobotThread( [this]{	startDriving();});
		robotThread.swap( newRobotThread);
	}
	/**
	 *
	 */
	void Robot::stopActing()
	{
		acting = false;
		driving = false;
		robotThread.join();
	}
	/**
	 *
	 */
	void Robot::startDriving()
	{
		driving = true;
		goal = RobotWorld::getRobotWorld().getGoal( "Goal");
		calculateRoute(goal);

		drive();
	}
	/**
	 *
	 */
	void Robot::stopDriving()
	{
		driving = false;
	}
	/**
	 *
	 */
	void Robot::startCommunicating()
	{
		if(!communicating)
		{
			communicating = true;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			if(Messaging::CommunicationService::getCommunicationService().isStopped())
			{
				TRACE_DEVELOP( "Restarting the Communication service");
				Messaging::CommunicationService::getCommunicationService().restart();
			}

			server = std::make_shared<Messaging::Server>(	static_cast<unsigned short>(std::stoi(localPort)),
															toPtr<Robot>());
			Messaging::CommunicationService::getCommunicationService().registerServer( server);
		}
	}
	/**
	 *
	 */
	void Robot::stopCommunicating()
	{
		if(communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::Client c1ient( 	"localhost",
										static_cast<unsigned short>(std::stoi(localPort)),
										toPtr<Robot>());
			Messaging::Message message( Messaging::StopCommunicatingRequest, "stop");
			c1ient.dispatchMessage( message);
		}
	}
	/**
	 *
	 */
	std::string Robot::getWorldInfo()
	{
		std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
		
		std::string worldInfo = "Walls";
		for(WallPtr wall : walls)
		{
			worldInfo += std::to_string(wall->getPoint1().x) + "," + std::to_string(wall->getPoint1().y) + "," +
						   std::to_string(wall->getPoint2().x) + "," + std::to_string(wall->getPoint2().y) + "_";
		}
		worldInfo = worldInfo.substr(0, worldInfo.size() - 1);
		worldInfo += ";";
		std::vector<GoalPtr> goals = RobotWorld::getRobotWorld().getGoals();
		worldInfo += "Goals";
		for(GoalPtr goal : goals)
		{
			worldInfo += std::to_string(goal->getPosition().x) + "," + std::to_string(goal->getPosition().y);
		}
		worldInfo += ";";
		std::vector<RobotPtr> robots = RobotWorld::getRobotWorld().getRobots();
		RobotPtr thisRobot = robots[0];
		wxPoint currentPos = thisRobot->getPosition();
		worldInfo += "Robot" + std::to_string(currentPos.x) + "," + std::to_string(currentPos.y) + "," + std::to_string(0.0) + "," + std::to_string(0.0);
		return worldInfo;
	}
	/**
	 *
	 */
	void Robot::sendWorldInfo()
	{
		std::string remoteIpAdres = "localhost";
		std::string remotePort = "12345";

		if (Application::MainApplication::isArgGiven( "-remote_ip"))
		{
			remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
		}
		if (Application::MainApplication::isArgGiven( "-remote_port"))
		{
			remotePort = Application::MainApplication::getArg( "-remote_port").value;
		}
		std::string wallsString = getWorldInfo();
		Application::Logger::log(wallsString);	
		Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(name);
		if(robot){
			Messaging::Client c1ient(remoteIpAdres, static_cast<unsigned short>(std::stoi(remotePort)),robot);
			Messaging::Message message( Messaging::SyncWorldRequest, wallsString);
			c1ient.dispatchMessage( message);
			
		}
		worldSyncer = true;
	}
	/**
	 *
	 */
	wxRegion Robot::getRegion() const
	{
		wxPoint translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return wxRegion( 4, translatedPoints); // @suppress("Avoid magic numbers")
	}
	/**
	 *
	 */
	bool Robot::intersects( const wxRegion& aRegion) const
	{
		wxRegion region = getRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	/**
	 *
	 */
	bool Robot::closeToOtherRobot(double maximumDistance) const
	{
		const std::vector<RobotPtr>& robots = RobotWorld::getRobotWorld().getRobots();

		for (RobotPtr robot : robots)
		{
			if(robot){
				double distanceToOtherRobot = Utils::Shape2DUtils::distance(position, robot->getPosition());
				if (distanceToOtherRobot < maximumDistance && distanceToOtherRobot != 0.0)
				{
					return true;
				}
			}
		}
		return false;	
	}
	/**
	 *
	 */
	void Robot::clearPath()
	{
		path.clear();
	}
	/**
	 *
	 */
	wxPoint Robot::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalFrontLeft( x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint frontLeft( static_cast<int>((originalFrontLeft.x - position.x) * std::cos( angle) - (originalFrontLeft.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalFrontLeft.y - position.y) * std::cos( angle) + (originalFrontLeft.x - position.x) * std::sin( angle) + position.y));

		return frontLeft;
	}
	/**
	 *
	 */
	wxPoint Robot::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalFrontRight( x + size.x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint frontRight( static_cast<int>((originalFrontRight.x - position.x) * std::cos( angle) - (originalFrontRight.y - position.y) * std::sin( angle) + position.x),
						  static_cast<int>((originalFrontRight.y - position.y) * std::cos( angle) + (originalFrontRight.x - position.x) * std::sin( angle) + position.y));

		return frontRight;
	}
	/**
	 *
	 */
	wxPoint Robot::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalBackLeft( x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint backLeft( static_cast<int>((originalBackLeft.x - position.x) * std::cos( angle) - (originalBackLeft.y - position.y) * std::sin( angle) + position.x),
						static_cast<int>((originalBackLeft.y - position.y) * std::cos( angle) + (originalBackLeft.x - position.x) * std::sin( angle) + position.y));

		return backLeft;
	}
	/**
	 *
	 */
	wxPoint Robot::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalBackRight( x + size.x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint backRight( static_cast<int>((originalBackRight.x - position.x) * std::cos( angle) - (originalBackRight.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalBackRight.y - position.y) * std::cos( angle) + (originalBackRight.x - position.x) * std::sin( angle) + position.y));

		return backRight;
	}
	/**
	 *
	 */
	void Robot::handleNotification()
	{
		//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

		static int update = 0;
		if ((++update % 200) == 0) // @suppress("Avoid magic numbers")
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	std::vector<int> Robot::getNumbersFromString(std::string stringWithNumbers)
	{
		std::vector<int> numbers;
		std::string result;
		std::istringstream numberStream(stringWithNumbers);

		while (std::getline(numberStream, result, ','))
		{
			numbers.push_back(stoi(result));
		}
		return numbers;
	}
	/**
	 *
	 */
	void Robot::syncWalls(std::string& wallString)
	{
		const std::regex word("(Walls)");
		std::stringstream wallRegexStream;
    	std::regex_replace(std::ostream_iterator<char>(wallRegexStream), wallString.begin(), wallString.end(), word, "");
		wallString = wallRegexStream.str();

		std::string result;
		std::istringstream wallStream(wallString);

		int x1, y1, x2, y2;

		while (std::getline(wallStream, result, '_'))
		{
			std::vector<int> coordinates = getNumbersFromString(result);
			
			x1 = coordinates[0];
			y1 = coordinates[1];
			x2 = coordinates[2];
			y2 = coordinates[3];

			RobotWorld::getRobotWorld().newWall(wxPoint(x1, y1), wxPoint(x2, y2));
			Application::Logger::log("Wall created at " + std::to_string(x1) + "," + std::to_string(y1) + " and " + std::to_string(x2) + "," + std::to_string(y2));
		}
	}
	/**
	 *
	 */
	void Robot::syncGoals(std::string& goalString)
	{
		const std::regex word("(Goals)");
		std::stringstream goalRegexStream;
    	std::regex_replace(std::ostream_iterator<char>(goalRegexStream), goalString.begin(), goalString.end(), word, "");
		goalString = goalRegexStream.str();

		std::string result;
		std::istringstream goalStream(goalString);

		int x, y;

		while (std::getline(goalStream, result, '_'))
		{
			std::vector<int> coordinates = getNumbersFromString(result);

			x = coordinates[0];
			y = coordinates[1];
			RobotWorld::getRobotWorld().newGoal("A", wxPoint(x, y));
		}
	}
	/**
	 *
	 */
	void Robot::syncRobot(std::string& robotString)
	{
		const std::regex word("(Robot)");
		std::stringstream robotRegexStream;
		std::regex_replace(std::ostream_iterator<char>(robotRegexStream), robotString.begin(), robotString.end(), word, "");
		robotString = robotRegexStream.str();

		std::string result;
		std::istringstream robotStream(robotString);

		int x, y, fx, fy;

		std::vector<int> coordinates = getNumbersFromString(robotString);
		x = coordinates[0];
		y = coordinates[1];
		fx = coordinates[2];
		fy = coordinates[3];
		const std::vector<RobotPtr>& robots = RobotWorld::getRobotWorld().getRobots();
		RobotPtr robot;
		if (robots.size() > 1)
		{
			robot = robots[1];
		}
		if(!robot)
		{
			Application::Logger::log("Robot not found");
		}
		else
		{
			Application::Logger::log("Moving robot: " + robot->asString());
			robot->setPosition(wxPoint(x, y), false);
			robot->setFront(BoundedVector(fx, fy), false);

			if (!driving)
			{
				robot->notifyObservers();
			}
		}
	}
	/**
	 *
	 */
	void Robot::addNewRobot(std::string& robotString)
	{
		const std::regex word("(Robot)");
		std::stringstream robotRegexStream;
		std::regex_replace(std::ostream_iterator<char>(robotRegexStream), robotString.begin(), robotString.end(), word, "");
		robotString = robotRegexStream.str();

		std::string result;
		std::istringstream robotStream(robotString);

		int x, y;

		std::vector<int> coordinates = getNumbersFromString(robotString);
		x = coordinates[0];
		y = coordinates[1];
		RobotWorld::getRobotWorld().newRobot("Robot", wxPoint(x, y));
	}
	/**
	 *
	 */
	void Robot::handleRequest( Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingRequest:
			{
				aMessage.setMessageType(Messaging::StopCommunicatingResponse);
				aMessage.setBody("StopCommunicatingResponse");
				// Handle the request. In the limited context of this works. I am not sure
				// whether this works OK in a real application because the handling is time sensitive,
				// i.e. 2 async timers are involved:
				// see CommunicationService::stopServer and Server::stopHandlingRequests
				Messaging::CommunicationService::getCommunicationService().stopServer(12345,true); // @suppress("Avoid magic numbers")

				break;
			}
			case Messaging::EchoRequest:
			{
				aMessage.setMessageType(Messaging::EchoResponse);
				aMessage.setBody( "Messaging::EchoResponse: " + aMessage.asString());
				break;
			}
			case Messaging::SyncWorldRequest:
			{	
				std::string syncmessage = aMessage.getBody();
				std::string response = getWorldInfo();

				aMessage.setMessageType(Messaging::SyncWorldResponse);
				aMessage.setBody(response);
				Application::Logger::log(syncmessage);
				std::string result = "";
				std::istringstream goalStream(syncmessage);
				while (std::getline(goalStream, result, ';'))
				{
					if (result.find("Walls") != std::string::npos)
					{
						syncWalls(result);
					}
					if (result.find("Goals") != std::string::npos)
					{
						syncGoals(result);
					}
					if (result.find("Robot") != std::string::npos)
					{
						if(RobotWorld::getRobotWorld().getRobots().size() == 1)
						{
							addNewRobot(result);
						}
						else{
							syncRobot(result);
						}
					}
				}

				break;
			}
			case::Messaging::SyncRobotRequest:
			{
				std::string syncmessage = aMessage.getBody();
				if (syncmessage.find("Robot") != std::string::npos)
				{
					syncRobot(syncmessage);
				}
				break;
			}
			case::Messaging::StartRobotRequest:
			{
				startActing();
				break;
			}
			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string(": default not implemented"));
				break;
			}
		}
	}
	/**
	 *
	 */
	void Robot::handleResponse( const Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingResponse:
			{
				//Messaging::CommunicationService::getCommunicationService().stop();
				break;
			}
			case Messaging::EchoResponse:
			{
				break;
			}
			case Messaging::SyncWorldResponse:
			{
				std::string syncmessage = aMessage.getBody();
				std::string result = "";
				std::istringstream goalStream(syncmessage);
				while (std::getline(goalStream, result, ';'))
				{
					if (result.find("Walls") != std::string::npos)
					{
						syncWalls(result);
					}
					if (result.find("Goals") != std::string::npos)
					{
						syncGoals(result);
					}
					if (result.find("Robot") != std::string::npos)
					{
						if(RobotWorld::getRobotWorld().getRobots().size() == 1)
						{
							addNewRobot(result);
						}
						else{
							syncRobot(result);
						}
					}
				}
				break;
			}
			case Messaging::SyncRobotResponse:
			{
				break;
			}
			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
				break;
			}
		}
	}
	/**
	 *
	 */
	std::string Robot::asString() const
	{
		std::ostringstream os;

		os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Robot::asDebugString() const
	{
		std::ostringstream os;

		os << "Robot:\n";
		os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

		return os.str();
	}
	/**
	 *
	 */
	void Robot::sendRobotPosMessage() 
	{
		std::string message = "Robot" + std::to_string(position.x) + "," + std::to_string(position.y) + "," + std::to_string(front.x) + "," + std::to_string(front.y);
		Messaging::Message msg( Messaging::SyncRobotRequest, message);
		Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(name);
		if(robot){	
			std::string remoteIpAdres = "localhost";
			std::string remotePort = "12345";
			
			if (Application::MainApplication::isArgGiven( "-remote_ip"))
			{
				remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
			}
			if (Application::MainApplication::isArgGiven( "-remote_port"))
			{
				remotePort = Application::MainApplication::getArg( "-remote_port").value;
			}
			if(robot){
				Messaging::Client c1ient(remoteIpAdres, static_cast<unsigned short>(std::stoi(remotePort)),robot);
				c1ient.dispatchMessage( msg);
			}
		}
	}
	/**
	 *
	 */
	void Robot::drive()
	{
		try
		{
			// The runtime value always wins!!
			speed = static_cast<float>(Application::MainApplication::getSettings().getSpeed());

			// Compare a float/double with another float/double: use epsilon...
			if (std::fabs(speed - 0.0) <= std::numeric_limits<float>::epsilon())
			{
				setSpeed(10.0, false); // @suppress("Avoid magic numbers")
			}

			// We use the real position for starters, not an estimated position.
			startPosition = position;

			unsigned pathPoint = 0;
			while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size()) // @suppress("Avoid magic numbers")
			{
				// Do the update
				const PathAlgorithm::Vertex& vertex = path[pathPoint+=static_cast<unsigned int>(speed)];
				front = BoundedVector( vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;
				sendRobotPosMessage();

				if (closeToOtherRobot(ROBOT_WARNING_DISTANCE) && !almostCollided)
				{
					almostCollided = true;
					if (worldSyncer)
					{
						driving = false;
						startDriving();
					}
					else
					{
						setSpeed(0.0, false);
					}
				}

				if (almostCollided && !closeToOtherRobot(ROBOT_RESTART_DISTANCE))
				{
					setSpeed(10.0, false);
				}

				// Stop on arrival or collision
				if (arrived(goal) || collision())
				{
					Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived or collision"));
					driving = false;
					almostCollided = false;
					worldSyncer = false;
				}

				notifyObservers();
				const std::vector<RobotPtr>& robots = RobotWorld::getRobotWorld().getRobots();
				RobotPtr remoteRobot;
				if (robots.size() > 1)
				{
					remoteRobot = robots[1];
				}
				
				if(remoteRobot)
				{
					remoteRobot->notifyObservers();
				}

				// If there is no sleep_for here the robot will immediately be on its destination....
				std::this_thread::sleep_for( std::chrono::milliseconds( 100)); // @suppress("Avoid magic numbers")

				// this should be the last thing in the loop
				if(driving == false)
				{
					break;
				}
			} // while
		}
		catch (std::exception& e)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": ") + e.what());
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": unknown exception"));
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}
	/**
	 *
	 */
	void Robot::calculateRoute(GoalPtr aGoal)
	{
		path.clear();
		if (aGoal)
		{
			// Turn off logging if not debugging AStar
			Application::Logger::setDisable();

			front = BoundedVector( aGoal->getPosition(), position);
			//handleNotificationsFor( astar);
			path = astar.search( position, aGoal->getPosition(), size);
			//stopHandlingNotificationsFor( astar);

			Application::Logger::setDisable( false);
		}
	}
	/**
	 *
	 */
	bool Robot::arrived(GoalPtr aGoal)
	{
		if (aGoal && intersects( aGoal->getRegion()))
		{
			return true;
		}
		return false;
	}
	/**
	 *
	 */
	bool Robot::collision()
	{
		wxPoint frontLeft = getFrontLeft();
		wxPoint frontRight = getFrontRight();
		wxPoint backLeft = getBackLeft();
		wxPoint backRight = getBackRight();

		const std::vector< WallPtr >& walls = RobotWorld::getRobotWorld().getWalls();
		for (WallPtr wall : walls)
		{
			if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) 	||
				Utils::Shape2DUtils::intersect( frontLeft, backLeft, wall->getPoint1(), wall->getPoint2())		||
				Utils::Shape2DUtils::intersect( frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
			{
				return true;
			}
		}
		const std::vector< RobotPtr >& robots = RobotWorld::getRobotWorld().getRobots();
		for (RobotPtr robot : robots)
		{
			if(robot){		
				if ( getObjectId() == robot->getObjectId())
				{
					continue;
				}
				if(intersects(robot->getRegion()))
				{
					return true;
				}
			}
		}
		return false;
	}

} // namespace Model

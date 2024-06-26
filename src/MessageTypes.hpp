#ifndef MESSAGETYPES_HPP_
#define MESSAGETYPES_HPP_

namespace Messaging
{
	/**
	 * @name The types of messages usable in the application
	 */
	//@{
	enum MessageType
	{
		CommunicationReadError,
		CommunicationWriteError,
		ServerSideExceptionRequest,
		ServerSideExceptionResponse,
		ApplicationExceptionRequest,
		ApplicationExceptionResponse,
		StopCommunicatingRequest,
		StopCommunicatingResponse,
		GenericRequest,
		GenericResponse,
		EchoRequest,
		EchoResponse,
		SyncWorldRequest,
		SyncWorldResponse,
		SyncRobotRequest,
		SyncRobotResponse,
		StartRobotRequest,
	};
	//@}
} /* namespace Messaging */

#endif // MESSAGETYPES_HPP_
